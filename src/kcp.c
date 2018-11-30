/**
 * @file
 * KCP protocol
 */

#include "kcp.h"
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>

#define MTU 1200
#define WND 512
#define KCP_INTERVAL_MS 10
#define UDP_MAX_PKT_SIZE 65536

#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

static uint32_t random_conv();

static void kcp_vlog(const char *fmt, ...)
{
	va_list argptr;
	va_start(argptr, fmt);
	vprintf(fmt, argptr);
	va_end(argptr);
}

static void kcp_log(const char *log, struct IKCPCB *kcp, void *user)
{
	KCPContext *s = (KCPContext *)user;
    kcp_vlog("KCP conv: %u, %s\n", s->kcp->conv, log);
}

static int kcp_onsend(const char *buf, int len, ikcpcb *kcp, void *user)
{
	int ret;
	KCPContext *s = (KCPContext*)user;
	ret = socket_sendto_by_time(s->sock, buf, len, 0, (struct sockaddr*)&s->addr, s->addrlen, s->timeout);
	if (ret < 0)
		kcp_vlog("kcp_onsend: %d, %d\n", ret, socket_geterror());
	return ret < 0 ? socket_geterror() : ret;
}

static int kcp_fifo_create(KCPFifo* s, int size)
{
	ring_buffer_alloc(&s->fifo, size);
	locker_create(&s->mutex);
	event_create(&s->cond);
	return 0;
}

static int kcp_fifo_destroy(KCPFifo* s)
{
	locker_destroy(&s->mutex);
	event_destroy(&s->cond);
	ring_buffer_free(&s->fifo);
	return 0;
}

/// @return read bytes
static int kcp_fifo_read(KCPFifo* s, void* buf, size_t size, int signal)
{
	size_t avail;

	locker_lock(&s->mutex);
	avail = ring_buffer_size(&s->fifo);
	if (avail < 1)
	{
		locker_unlock(&s->mutex);
		return 0;
	}

	avail = avail < size ? avail : size;
	ring_buffer_read(&s->fifo, buf, avail);

	if (signal)
		event_signal(&s->cond);
	locker_unlock(&s->mutex);
	return avail;
}

static int kcp_fifo_write(KCPFifo* s, const void* buf, size_t size, int signal)
{
	size_t space;

	locker_lock(&s->mutex);
	space = ring_buffer_space(&s->fifo);
	if (space < 1)
	{
		locker_unlock(&s->mutex);
		return 0;
	}

	size = space < size ? space : size;
	ring_buffer_write(&s->fifo, buf, size); /* the data */

	if (signal)
		event_signal(&s->cond);
	locker_unlock(&s->mutex);
	return size;
}

static int kcp_try_read_all(KCPContext *s)
{
	int ret;
	int size;

	while(1)
	{
		size = ikcp_peeksize(s->kcp);
		assert(size < (int)s->rfifo.fifo.capacity);
		if (size <= 0 || (int)ring_buffer_space(&s->rfifo.fifo) < size)
			break; // no more data or no space

		size = FFMIN(sizeof(s->data), size);
		ret = ikcp_recv(s->kcp, (char*)s->data, size);
		if (ret < 0)
			return ret;

		size = kcp_fifo_write(&s->rfifo, s->data, ret, 1);
		if (size != ret)
			return -ETIMEDOUT;
	}

	return 0;
}

static int kcp_try_write_all(KCPContext *s)
{
	int delay;
	int size, ret;

	while (1)
	{
		delay = ikcp_waitsnd(s->kcp);
		if (delay >= s->wnd_send)
			break; // too many packet delay

		// fixed: ikcp_send if (count >= IKCP_WND_RCV) return -2;
		size = FFMIN(sizeof(s->data), s->kcp->mss * 100/*IKCP_WND_RCV*/);
		size = FFMIN(size, (s->wnd_send - delay) * s->kcp->mss);
		size = kcp_fifo_read(&s->wfifo, s->data, size, 1);
		if (0 == size)
			break; // no more data to send

		ret = ikcp_send(s->kcp, (const char*)s->data, size);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int STDCALL kcp_process(void *_URLContext)
{
	int size, ret, count;
	KCPContext *s = _URLContext;
	struct sockaddr_storage addr;
	socklen_t addrlen;

	count = 0;
	while (s->thread_started != 2)
	{
		//TODO: use ikcp_check calc next update time
		
		// read MSG/ACK
		ret = socket_select_read(s->sock, KCP_INTERVAL_MS);
		if (ret > 0)
		{
			count = 0;
			addrlen = sizeof(addr);
			size = socket_recvfrom(s->sock, s->data, sizeof(s->data), 0, (struct sockaddr*)&addr, &addrlen);
			if (size > 0)
			{
				s->kcp->current = (IUINT32)system_clock();
				ret = ikcp_input(s->kcp, (const char*)s->data, size);
				if (ret >= 0)
				{
					// kcp recv buffer -> fifo
					ret = kcp_try_read_all(s);
				}

				if (ret < 0)
					break;
			}
		}

		if (++count * KCP_INTERVAL_MS > s->timeout)
		{
			ret = -ETIMEDOUT;
			break;
		}

		// fifo -> kcp send buffer
		ret = kcp_try_write_all(s);
		if (ret < 0)
			break;

		ikcp_update(s->kcp, (IUINT32)system_clock());
	}

	s->thread_errno = ret;
	kcp_vlog("[%u] kcp_process exit: %d\n", (IUINT32)system_clock(), ret);
	return 0;
}

int kcp_close(KCPContext *s)
{
	s->thread_started = 2;
	event_signal(&s->rfifo.cond);
	event_signal(&s->wfifo.cond);
	thread_destroy(s->kcp_thread);

	socket_close(s->sock);
	s->sock = socket_invalid;
	if (s->kcp)
	{
		ikcp_release(s->kcp);
		s->kcp = NULL;
	}
	kcp_fifo_destroy(&s->rfifo);
	kcp_fifo_destroy(&s->wfifo);

	//free(s);
	return 0;
}

void kcp_init(KCPContext* s)
{
	s->mtu_size = MTU;
	s->wnd_send = WND;
	s->wnd_recv = WND;
	s->timeout = 5 * 1000;
	s->fast_ack = 3;
	s->cong_lost = 5;
	s->cong_band = 200 * 1000 / 8;
	s->cong_incr = 1.25f;
	s->cong_decr = 0.75f;

	s->rfifo_size = 512 * 1024;
	s->wfifo_size = 512 * 1024;

	s->dnsTime = 0;
	s->sock = socket_invalid;
}

/* put it in kcp context */
/* return non zero if error */
int kcp_open(KCPContext* s, const char *host, int port)
{
	int ret;
	size_t rbsize;
	size_t wbsize;
	uint64_t clock;
	
	clock = system_clock();
	ret = socket_addr_from(&s->addr, &s->addrlen, host, port);
	s->dnsTime = system_clock() - clock;
	if (0 != ret)
		return ret;

	s->sock = socket_udp();
	s->conv = random_conv();
	socket_setdontfrag(s->sock, 1); // don't fragment

    s->kcp = ikcp_create(s->conv, s);
    ikcp_setoutput(s->kcp, kcp_onsend);
    ikcp_nodelay(s->kcp, 0, KCP_INTERVAL_MS, s->fast_ack, 1);
	ikcp_congestion(s->kcp, s->cong_band, s->cong_lost, s->cong_incr, s->cong_decr);
    ikcp_wndsize(s->kcp, s->wnd_send, s->wnd_recv);
    ikcp_setmtu(s->kcp, s->mtu_size);
    s->kcp->stream = 1;
	s->kcp->logmask = -1;
	s->kcp->writelog = kcp_log;
    kcp_vlog("KCP conv: %u, wnd_send: %d, wnd_recv: %d, mtu: %d\n", s->kcp->conv, s->kcp->snd_wnd, s->kcp->rcv_wnd, s->kcp->mtu);
	kcp_vlog("KCP nodelay: %d, resend: %d, nc: %d, interval: %dms\n", s->kcp->nodelay, s->kcp->fastresend, s->kcp->nocwnd, s->kcp->interval);
	kcp_vlog("KCP congest: %d, incr: %f, decr: %f\n", s->kcp->cong_lost, s->kcp->cong_incr, s->kcp->cong_decr);
	socket_getrecvbuf(s->sock, &rbsize);
	socket_getsendbuf(s->sock, &wbsize);
	kcp_vlog("KCP socket recv buffer size: %d, send buffer size: %d\n", rbsize, wbsize);

	kcp_fifo_create(&s->rfifo, s->rfifo_size);
	kcp_fifo_create(&s->wfifo, s->wfifo_size);
	kcp_vlog("KCP fifo recv: %d, send: %d\n", s->rfifo_size, s->wfifo_size);

	s->thread_started = 1;
	ret = thread_create(&s->kcp_thread, kcp_process, s);
	if (ret != 0) {
		kcp_vlog("pthread_create failed : %s\n", strerror(ret));
		goto fail;
	}

    return 0;

fail:
    kcp_close(s);
    return EIO;
}

int kcp_read(KCPContext *s, uint8_t *buf, int size, int nonblock)
{
	int ret = -EAGAIN;
	KCPFifo *rx = &s->rfifo;
	
	while (0 == s->thread_errno && s->thread_started != 2)
	{
		ret = kcp_fifo_read(rx, buf, size, 0);
		if (0 != ret)
			return ret;

		if (nonblock)
			return 0;

		ret = event_timewait(&rx->cond, s->timeout);
		if (0 != ret)
			return -ETIMEDOUT;
	}

	return s->thread_errno;
}

// KCP streaming send data
int kcp_write(KCPContext *s, const uint8_t *buf, int size)
{
	int r;
	KCPFifo *tx = &s->wfifo;
	const uint8_t* ptr, *end;

	ptr = buf;
	end = buf + size;
	while (0 == s->thread_errno && s->thread_started != 2)
	{
		r = kcp_fifo_write(tx, ptr, end - ptr, 0);
		if (r < 0)
			return r;

		ptr += r;
		if (ptr >= end)
			return 0;

		if(0 != event_timewait(&tx->cond, s->timeout))
			return -ETIMEDOUT;
	}

	return s->thread_errno;
}

static int read_random(uint32_t *dst, const char *file)
{
	int fd = open(file, O_RDONLY);
	int err = -1;
	if (fd == -1)
		return -1;
	err = read(fd, dst, sizeof(*dst));
	close(fd);
	return err;
}
static uint32_t random_conv()
{
	uint32_t seed;

#if defined(_WIN32) || defined(_WIN64)
	HCRYPTPROV provider;
	if (CryptAcquireContext(&provider, NULL, NULL, PROV_RSA_FULL,
		CRYPT_VERIFYCONTEXT | CRYPT_SILENT)) {
		BOOL ret = CryptGenRandom(provider, sizeof(seed), (PBYTE)&seed);
		CryptReleaseContext(provider, 0);
		if (ret)
			return seed;
	}
#endif

	if (read_random(&seed, "/dev/urandom") == sizeof(seed))
		return seed;
	if (read_random(&seed, "/dev/random") == sizeof(seed))
		return seed;
	return rand();
}
