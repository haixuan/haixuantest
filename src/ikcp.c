//=====================================================================
//
// KCP - A Better ARQ Protocol Implementation
// skywind3000 (at) gmail.com, 2010-2011
//  
// Features:
// + Average RTT reduce 30% - 40% vs traditional ARQ like tcp.
// + Maximum RTT reduce three times vs tcp.
// + Lightweight, distributed as a single source file.
//
//=====================================================================
#include "ikcp.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>
#include "sys/system.h"

#define KCP_REPORT_WINDOW			1000 // ms
#define KCP_RTT_FILTER_WINDOW		5000 // ms
#define KCP_BANDWIDTH_FILTER_WINDOW 5000 // ms
#define KCP_TIMEOUT_THRESHOLD		500 //ms
#define KCP_INTERVAL_MAX 40 //5000 ms
#define KCP_INTERVAL_MIN 10 // ms

#define KCP_CONGEST_INTERVAL 100

#define KCP_FAST_RETRANSMISSION 3
#define KCP_TIMEOUT_DELAY 1000

//=====================================================================
// KCP BASIC
//=====================================================================
const IUINT32 IKCP_RTO_NDL = 30;		// no delay min rto
const IUINT32 IKCP_RTO_MIN = 50; // 100;		// normal min rto
const IUINT32 IKCP_RTO_DEF = 200;
const IUINT32 IKCP_RTO_MAX = 4000;// 60000;
const IUINT32 IKCP_CMD_PUSH = 81;		// cmd: push data
const IUINT32 IKCP_CMD_ACK  = 82;		// cmd: ack
const IUINT32 IKCP_CMD_WASK = 83;		// cmd: window probe (ask)
const IUINT32 IKCP_CMD_WINS = 84;		// cmd: window size (tell)
const IUINT32 IKCP_ASK_SEND = 1;		// need to send IKCP_CMD_WASK
const IUINT32 IKCP_ASK_TELL = 2;		// need to send IKCP_CMD_WINS
const IUINT32 IKCP_WND_SND = 32;
const IUINT32 IKCP_WND_RCV = 128;       // must >= max fragment size
const IUINT32 IKCP_MTU_DEF = 1400;
const IUINT32 IKCP_ACK_FAST	= 3;
const IUINT32 IKCP_INTERVAL	= 100;
const IUINT32 IKCP_OVERHEAD = 24;
const IUINT32 IKCP_DEADLINK = 20;
const IUINT32 IKCP_THRESH_INIT = 2;
const IUINT32 IKCP_THRESH_MIN = 2;
const IUINT32 IKCP_PROBE_INIT = 7000;		// 7 secs to probe window size
const IUINT32 IKCP_PROBE_LIMIT = 120000;	// up to 120 secs to probe window

// return a > b ? 1 : 0
static inline int __gt__(IUINT32 a, IUINT32 b) {
	return (a > b && a + 0xFFFFFF > b) ? 1 : 0;
}

static inline IUINT32 __max__(IUINT32 a, IUINT32 b) {
	return a >= b ? a : b;
}

// write log
void ikcp_log(ikcpcb *kcp, int mask, const char *fmt, ...)
{
#if 0
	char buffer[1024];
	va_list argptr;
	if ((mask & kcp->logmask) == 0 || kcp->writelog == 0) return;
	va_start(argptr, fmt);
	vsprintf(buffer, fmt, argptr);
	va_end(argptr);
	kcp->writelog(buffer, kcp, kcp->user);
#endif
}

static void ikcp_trace(ikcpcb *kcp, const char *fmt, ...)
{
//	char buffer[1024];
	va_list argptr;
	va_start(argptr, fmt);

	if ((IKCP_LOG_OUTPUT & kcp->logmask) == 0 || kcp->writelog == 0) return;

#if defined(DEBUG) || defined(_DEBUG)
	vprintf(fmt, argptr);
#endif

	va_end(argptr);
}

static void kcp_notify(struct IKCPCB* kcp, const char *fmt, ...)
{
	char log[1024];
	va_list argptr;
	va_start(argptr, fmt);
	vsnprintf(log, sizeof(log), fmt, argptr);
	va_end(argptr);

	if(kcp->notify)
		kcp->notify(0, log, kcp->notifyparam);

#if defined(DEBUG) || defined(_DEBUG)
	if ((IKCP_LOG_OUTPUT & kcp->logmask) == 0 || kcp->writelog == 0) 
		return;
	printf("%s\n", log);
#endif
}

static inline IUINT32 kcp_get_inflight(struct IKCPCB* kcp)
{
	IUINT32 inflight = 0;
	struct IQUEUEHEAD *p;
	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = p->next) {
		struct IKCPSEG *seg = iqueue_entry(p, struct IKCPSEG, node);
		inflight += (seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/) * seg->xmit;
	}
	return inflight;
}

static inline IUINT32 kcp_get_unsend(struct IKCPCB* kcp)
{
	IUINT32 unsend = 0;
	struct IQUEUEHEAD *p;
	for (p = kcp->snd_queue.next; p != &kcp->snd_queue; p = p->next) {
		struct IKCPSEG *seg = iqueue_entry(p, struct IKCPSEG, node);
		unsend += seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/;
	}
	return unsend;
}

static inline void kcp_update_bitrate(struct IKCPCB* kcp, int len)
{
	uint32_t bitrate;
	kcp->bitrate_bytes += len;

	if (0 == kcp->bitrate_clock)
	{
		kcp->bitrate_clock = kcp->current;
	}
	else if(kcp->bitrate_clock + 10 * 1000 /*10s*/ < kcp->current)
	{
		bitrate = kcp->bitrate_bytes * 1000 / (kcp->current - kcp->bitrate_clock);
		minmax_running_max(&kcp->bitrate, 60 * 1000 /*1-minutes*/, kcp->current, bitrate);
		kcp->bitrate_clock = kcp->current;
		kcp->bitrate_bytes = 0;
	}
}

static inline void kcp_update_bandwidth(struct IKCPCB* kcp, const struct IKCPSEG* seg, IUINT32 current)
{
	IUINT32 rtt;
	IUINT32 bandwidth;

	if (1 != seg->xmit || current <= seg->ts)
		return;

	rtt = current - seg->ts;
	//ikcp_trace(kcp, "[%u] updatebw %d, delivered_sn: %d, kcp->delivered: %d, seg->delivered: %d, rtt: %d(%d), bandwidth: %.1fkbps(%.1fkbps)\n", current, seg->sn, kcp->delivered_sn, kcp->delivered, seg->delivered, current - seg->ts, minmax_get(&kcp->rtt), ((kcp->delivered + (seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/) * seg->xmit) - seg->delivered) * 8.0/rtt, minmax_get(&kcp->bandwidth) * 8 / 1000.0);
//	if (rtt < minmax_get(&kcp->rtt) /*|| seg->sn <= kcp->delivered_sn*/) return;

	kcp->delivered_sn = seg->sn;
	kcp->delivered += (seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/) /** seg->xmit*/;
	bandwidth = (IUINT32)((kcp->delivered - seg->delivered) * 1000 / rtt);
	kcpcongest_update(&kcp->cband, bandwidth, current);
	netstat_update_bandwidth(&kcp->netstat, bandwidth);
	// don't update bandwidth in 1-rtt after congest avoid 
	if (__gt__(seg->txsn+1, kcp->cong_band_sn))
		minmax_running_max(&kcp->bandwidth, KCP_BANDWIDTH_FILTER_WINDOW, current, bandwidth);
}

static inline void kcp_update_rtt(struct IKCPCB* kcp, const struct IKCPSEG* seg, IUINT32 current)
{
	IUINT32 rtt;
	if (1 != seg->xmit || current <= seg->ts)
		return;

	rtt = current - seg->ts;
	//if(rtt * 2 < kcp->rx_rto || kcp->rx_rto * 2 < rtt)
	//if (rtt * 2 < minmax_get(&kcp->rtt) && rtt + 20 < minmax_get(&kcp->rtt))
	//	return; // filter 
	//if (rtt > 500)
	//	return;
	minmax_running_min(&kcp->rtt, KCP_RTT_FILTER_WINDOW, current, rtt);
	netstat_update_rtt(&kcp->netstat, rtt);
	kcpcongest_update(&kcp->crtt, rtt, current);
}

static inline IUINT32 kcp_get_bdp(struct IKCPCB* kcp)
{
	IUINT32 bdp, rtt, bandwidth;

	rtt = minmax_get(&kcp->rtt);
	rtt = rtt < kcp->interval ? kcp->interval : rtt;

	bandwidth = minmax_get(&kcp->bandwidth);
	//bitrate = minmax_get(&kcp->bitrate);
	//if (bandwidth < bitrate * 12 / 10)
	//	bandwidth = bitrate * 12 / 10;

	bdp = bandwidth * rtt / 1000;
	if (1 == kcp->slowstart)
		bdp = bdp * 2;
	//else if (bandwidth < kcp_get_unsend(kcp) || bandwidth < minmax_get(&kcp->bitrate))
	//	bdp = bdp * 3 / 2; // bdp * 1.5
	else
		bdp = (IUINT32)(bdp * kcp->cong_incr); // bdp * 1.25

	if (bdp < 5 * 1000)
		bdp = 5 * 1000;

	return bdp;
}

static void kcp_update_threshold(struct IKCPCB* kcp, IUINT32 current)
{
	IUINT32 interval;
	IUINT32 bdp, bandwidth;

	bandwidth = minmax_get(&kcp->bandwidth);
	
	if(bandwidth < kcp->cong_band){
		bandwidth = kcp->cong_band;
	}

	interval = current - kcp->last_send_ts;
	kcp->last_send_ts = current;

	// fixed udp/tcp send block timeout
	if(interval > kcp->interval * 2)
		interval = kcp->interval * 2;

	bdp = bandwidth * interval / 1000;
	if (1 == kcp->slowstart)
		bdp = bdp * 2;
	//else if (bandwidth < kcp_get_unsend(kcp) || bandwidth < minmax_get(&kcp->bitrate))
	//	bdp = bdp * 3 / 2; // bdp * 1.5
	else
		bdp = (IUINT32)(bdp * kcp->cong_incr);

	kcp->total_bytes += bdp;
}

static int kcp_pacing(struct IKCPCB* kcp, const struct IKCPSEG* seg)
{
	// kcp pacing
	IUINT32 size;
	size = seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/;
	if (kcp->total_send + size > kcp->total_bytes) {
		//ikcp_trace(kcp, "[%u] total %"PRIu64", send: %"PRIu64", data: %d, segment: %d, rto: %d, rtt: %d, bandwidth: %.1fkbps\n", kcp->current, kcp->total_bytes, kcp->total_send, seg->len, seg->sn, kcp->rx_rto, minmax_get(&kcp->rtt), BANDWIDTH(minmax_get(&kcp->bandwidth)));
		return 0;
	}
	kcp->total_send += size;
	return 1;
}

static void kcp_onack(struct IKCPCB* kcp, const struct IKCPSEG* seg, IUINT32 current)
{
	IUINT32 rtt, bandwidth;
	rtt = minmax_get(&kcp->rtt);
	bandwidth = minmax_get(&kcp->bandwidth);

	if (0 == kcp->netstat.clock)
	{
		kcp->netstat.clock = current;
		kcp->netstat.sn = seg->sn;
	}
	else if (current - kcp->netstat.clock >= KCP_REPORT_WINDOW)
	{
		kcp_notify(kcp, "[%u] sn: %u, rto: %d -> %d, rtt: (%u)%u -> %u, bandwidth: (%.1fkbps)%.1fkbps -> %.1fkbps, count: %u, timeout: %u, fastack: %u", (unsigned int)kcp->netstat.clock, (unsigned int)kcp->netstat.sn, kcp->netstat.rto_min, kcp->netstat.rto_max, (unsigned int)rtt, (unsigned int)kcp->netstat.rtt_min, (unsigned int)kcp->netstat.rtt_max, BANDWIDTH(bandwidth), BANDWIDTH(kcp->netstat.bandwidth_min), BANDWIDTH(kcp->netstat.bandwidth_max), kcp->netstat.count, kcp->netstat.lost, kcp->netstat.fastack);
		netstat_reset(&kcp->netstat);
	}

	if (1 == seg->xmit && current > seg->ts)
	{
		rtt = current - seg->ts;
		bandwidth = (IUINT32)((kcp->delivered - seg->delivered + (seg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/) /** seg->xmit*/) * 1000 / rtt);
	}
	kcp_update_rtt(kcp, seg, current);
	kcp_update_bandwidth(kcp, seg, current);
	ikcp_trace(kcp, "[%u] acked %u, rto: %d, rtt: %u -> %u, bandwidth: %.1fkbps -> %.1fkbps, bitrate: %u\n", (unsigned int)current, (unsigned int)seg->sn, (unsigned int)kcp->rx_rto, (unsigned int)rtt, (unsigned int)minmax_get(&kcp->rtt), BANDWIDTH(bandwidth), BANDWIDTH(minmax_get(&kcp->bandwidth)), (unsigned int)minmax_get(&kcp->bitrate));
}

static void kcp_congest_avoid(struct IKCPCB* kcp, IUINT32 current)
{
	IUINT32 resent;
	double factor = 0;
	double kcplost = 0.0;
	unsigned int lost = 0;
	const char* lostinfo;
	struct IQUEUEHEAD *p;
	struct IKCPSEG *segment;

	resent = (kcp->fastresend > 0) ? (IUINT32)kcp->fastresend : 0xffffffff;
	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = p->next) {
		segment = iqueue_entry(p, struct IKCPSEG, node);
		if (__gt__(kcp->cong_avoid_sn, segment->txsn))
			continue;
		if(__gt__(current, segment->resendts) || segment->fastack >= resent)
			lost += segment->cxmit; // mark as timeout
	}
	kcplost = kcplost_check(&kcp->clost, lost);
	lostinfo = kcplost_print(&kcp->clost);

	// 1. packet lost percent in BDP > 30%
	// 2. rtt monotonic increase (3-rtt)
	if (kcplost > 0.3) {
		kcp->cong_avoid_sn = kcp->txsn;
		kcplost_reset(&kcp->clost);
		factor = 0.5;
	} else if (kcpcongest_check(&kcp->crtt, kcp->slowstart ? 15 : 7)) {
		factor = kcp->cong_decr;
	} else {
		return;
	}

	if (kcp->slowstart)
	{
		kcp->slowstart = 0;
		kcp_notify(kcp, "[%u] slowstart %d", kcp->current, kcp->snd_nxt);
	}

	// Congestion Avoidance
	kcp_notify(kcp, "[%u] lost %u, rtt: %u, rto: %d, bandwidth: %.1fkbps ==> %.1fkbps, unack: %.1f%%(%s), rtt: (%s)ms, band: (%s)kbps", (unsigned int)kcp->current, (unsigned int)kcp->snd_nxt, (unsigned int)minmax_get(&kcp->rtt), (int)kcp->rx_rto, BANDWIDTH(minmax_get(&kcp->bandwidth)), BANDWIDTH(minmax_get(&kcp->bandwidth) * factor), kcplost * 100, lostinfo, kcpcongest_print(&kcp->crtt, 0), kcpcongest_print(&kcp->cband, 1));
	minmax_reset(&kcp->bandwidth, kcp->current, (IUINT32)__max__(kcp->cong_band, minmax_get(&kcp->bandwidth) * factor));
	//minmax_reset(&kcp->rtt, kcp->current - KCP_RTT_FILTER_WINDOW * 2, minmax_get(&kcp->rtt));

	kcp->cong_band_sn = kcp->txsn;
	kcpcongest_reset(&kcp->crtt, KCP_CONGEST_INTERVAL, kcp->current);
	kcpcongest_reset(&kcp->cband, KCP_CONGEST_INTERVAL, kcp->current);
}

//---------------------------------------------------------------------
// encode / decode
//---------------------------------------------------------------------

/* encode 8 bits unsigned int */
static inline char *ikcp_encode8u(char *p, unsigned char c)
{
	*(unsigned char*)p++ = c;
	return p;
}

/* decode 8 bits unsigned int */
static inline const char *ikcp_decode8u(const char *p, unsigned char *c)
{
	*c = *(unsigned char*)p++;
	return p;
}

/* encode 16 bits unsigned int (lsb) */
static inline char *ikcp_encode16u(char *p, unsigned short w)
{
#if IWORDS_BIG_ENDIAN
	*(unsigned char*)(p + 0) = (w & 255);
	*(unsigned char*)(p + 1) = (w >> 8);
#else
	*(unsigned short*)(p) = w;
#endif
	p += 2;
	return p;
}

/* decode 16 bits unsigned int (lsb) */
static inline const char *ikcp_decode16u(const char *p, unsigned short *w)
{
#if IWORDS_BIG_ENDIAN
	*w = *(const unsigned char*)(p + 1);
	*w = *(const unsigned char*)(p + 0) + (*w << 8);
#else
	*w = *(const unsigned short*)p;
#endif
	p += 2;
	return p;
}

/* encode 32 bits unsigned int (lsb) */
static inline char *ikcp_encode32u(char *p, IUINT32 l)
{
#if IWORDS_BIG_ENDIAN
	*(unsigned char*)(p + 0) = (unsigned char)((l >>  0) & 0xff);
	*(unsigned char*)(p + 1) = (unsigned char)((l >>  8) & 0xff);
	*(unsigned char*)(p + 2) = (unsigned char)((l >> 16) & 0xff);
	*(unsigned char*)(p + 3) = (unsigned char)((l >> 24) & 0xff);
#else
	*(IUINT32*)p = l;
#endif
	p += 4;
	return p;
}

/* decode 32 bits unsigned int (lsb) */
static inline const char *ikcp_decode32u(const char *p, IUINT32 *l)
{
#if IWORDS_BIG_ENDIAN
	*l = *(const unsigned char*)(p + 3);
	*l = *(const unsigned char*)(p + 2) + (*l << 8);
	*l = *(const unsigned char*)(p + 1) + (*l << 8);
	*l = *(const unsigned char*)(p + 0) + (*l << 8);
#else 
	*l = *(const IUINT32*)p;
#endif
	p += 4;
	return p;
}

static inline IUINT32 _imin_(IUINT32 a, IUINT32 b) {
	return a <= b ? a : b;
}

static inline IUINT32 _imax_(IUINT32 a, IUINT32 b) {
	return a >= b ? a : b;
}

static inline IUINT32 _ibound_(IUINT32 lower, IUINT32 middle, IUINT32 upper) 
{
	return _imin_(_imax_(lower, middle), upper);
}

static inline long _itimediff(IUINT32 later, IUINT32 earlier) 
{
	return ((IINT32)(later - earlier));
}

//---------------------------------------------------------------------
// manage segment
//---------------------------------------------------------------------
typedef struct IKCPSEG IKCPSEG;

static void* (*ikcp_malloc_hook)(size_t) = NULL;
static void (*ikcp_free_hook)(void *) = NULL;

// internal malloc
static void* ikcp_malloc(size_t size) {
	if (ikcp_malloc_hook) 
		return ikcp_malloc_hook(size);
	return malloc(size);
}

// internal free
static void ikcp_free(void *ptr) {
	if (ikcp_free_hook) {
		ikcp_free_hook(ptr);
	}	else {
		free(ptr);
	}
}

// redefine allocator
void ikcp_allocator(void* (*new_malloc)(size_t), void (*new_free)(void*))
{
	ikcp_malloc_hook = new_malloc;
	ikcp_free_hook = new_free;
}

// allocate a new kcp segment
static IKCPSEG* ikcp_segment_new(ikcpcb *kcp, int size)
{
	return (IKCPSEG*)ikcp_malloc(sizeof(IKCPSEG) + size);
}

// delete a segment
static void ikcp_segment_delete(ikcpcb *kcp, IKCPSEG *seg)
{
	ikcp_free(seg);
}

// check log mask
static int ikcp_canlog(const ikcpcb *kcp, int mask)
{
	if ((mask & kcp->logmask) == 0 || kcp->writelog == NULL) return 0;
	return 1;
}

// output segment
static int ikcp_output(ikcpcb *kcp, const void *data, int size)
{
	assert(kcp);
	assert(kcp->output);
	if (ikcp_canlog(kcp, IKCP_LOG_OUTPUT)) {
		ikcp_log(kcp, IKCP_LOG_OUTPUT, "[RO] %ld bytes", (long)size);
	}
	if (size == 0) return 0;
	return kcp->output((const char*)data, size, kcp, kcp->user);
}

// output queue
static void ikcp_qprint(const char *name, const struct IQUEUEHEAD *head)
{
#if 0
	const struct IQUEUEHEAD *p;
	printf("<%s>: [", name);
	for (p = head->next; p != head; p = p->next) {
		const IKCPSEG *seg = iqueue_entry(p, const IKCPSEG, node);
		printf("(%lu %d)", (unsigned long)seg->sn, (int)(seg->ts % 10000));
		if (p->next != head) printf(",");
	}
	printf("]\n");
#endif
}


//---------------------------------------------------------------------
// create a new kcpcb
//---------------------------------------------------------------------
ikcpcb* ikcp_create(IUINT32 conv, void *user)
{
	ikcpcb *kcp = (ikcpcb*)ikcp_malloc(sizeof(struct IKCPCB));
	if (kcp == NULL) return NULL;
	kcp->conv = conv;
	kcp->user = user;
	kcp->snd_una = 0;
	kcp->snd_nxt = 0;
	kcp->rcv_nxt = 0;
	kcp->ts_recent = 0;
	kcp->ts_lastack = 0;
	kcp->ts_probe = 0;
	kcp->probe_wait = 0;
	kcp->snd_wnd = IKCP_WND_SND;
	kcp->rcv_wnd = IKCP_WND_RCV;
	kcp->rmt_wnd = IKCP_WND_RCV;
	kcp->cwnd = 0;
	kcp->incr = 0;
	kcp->probe = 0;
	kcp->mtu = IKCP_MTU_DEF;
	kcp->mss = kcp->mtu - IKCP_OVERHEAD;
	kcp->stream = 0;

	kcp->buffer = (char*)ikcp_malloc((kcp->mtu + IKCP_OVERHEAD) * 3);
	if (kcp->buffer == NULL) {
		ikcp_free(kcp);
		return NULL;
	}

	iqueue_init(&kcp->snd_queue);
	iqueue_init(&kcp->rcv_queue);
	iqueue_init(&kcp->snd_buf);
	iqueue_init(&kcp->rcv_buf);
	kcp->nrcv_buf = 0;
	kcp->nsnd_buf = 0;
	kcp->nrcv_que = 0;
	kcp->nsnd_que = 0;
	kcp->state = 0;
	kcp->acklist = NULL;
	kcp->ackblock = 0;
	kcp->ackcount = 0;
	kcp->rx_srtt = 0;
	kcp->rx_rttval = 0;
	kcp->rx_rto = IKCP_RTO_DEF;
	kcp->rx_minrto = IKCP_RTO_MIN;
	kcp->current = 0;
	kcp->interval = IKCP_INTERVAL;
	kcp->ts_flush = IKCP_INTERVAL;
	kcp->nodelay = 0;
	kcp->updated = 0;
	kcp->logmask = 0;
	kcp->ssthresh = IKCP_THRESH_INIT;
	kcp->fastresend = 0;
	kcp->nocwnd = 0;
	kcp->xmit = 0;
	kcp->dead_link = IKCP_DEADLINK;
	kcp->output = NULL;
	kcp->writelog = NULL;

	kcp->cong_lost = 5;
	kcp->cong_band = 200 * 1000 / 8; // at least 200kbps
	kcp->cong_incr = 1.25;
	kcp->cong_decr = 0.75;

	kcp->slowstart = 1;
	kcp->delivered = 0;
	kcp->delivered_sn = 0;
	kcp->bitrate_bytes = 0;
	kcp->bitrate_clock = 0;
	memset(&kcp->rtt, 0, sizeof(kcp->rtt));
	memset(&kcp->bitrate, 0, sizeof(kcp->bitrate));
	memset(&kcp->bandwidth, 0, sizeof(kcp->bandwidth));

	kcp->txsn = 0;
	kcp->last_send_ts = 0;
	kcp->total_bytes = 0;
	kcp->total_send = 0;
	kcp->cong_band_sn = 0;
	kcp->cong_avoid_sn = 0;
	netstat_reset(&kcp->netstat);
	kcplost_reset(&kcp->clost);
	kcpcongest_reset(&kcp->crtt, KCP_CONGEST_INTERVAL, 0);
	kcpcongest_reset(&kcp->cband, KCP_CONGEST_INTERVAL, 0);
	return kcp;
}


//---------------------------------------------------------------------
// release a new kcpcb
//---------------------------------------------------------------------
void ikcp_release(ikcpcb *kcp)
{
	assert(kcp);
	if (kcp) {
		IKCPSEG *seg;
		while (!iqueue_is_empty(&kcp->snd_buf)) {
			seg = iqueue_entry(kcp->snd_buf.next, IKCPSEG, node);
			iqueue_del(&seg->node);
			ikcp_segment_delete(kcp, seg);
		}
		while (!iqueue_is_empty(&kcp->rcv_buf)) {
			seg = iqueue_entry(kcp->rcv_buf.next, IKCPSEG, node);
			iqueue_del(&seg->node);
			ikcp_segment_delete(kcp, seg);
		}
		while (!iqueue_is_empty(&kcp->snd_queue)) {
			seg = iqueue_entry(kcp->snd_queue.next, IKCPSEG, node);
			iqueue_del(&seg->node);
			ikcp_segment_delete(kcp, seg);
		}
		while (!iqueue_is_empty(&kcp->rcv_queue)) {
			seg = iqueue_entry(kcp->rcv_queue.next, IKCPSEG, node);
			iqueue_del(&seg->node);
			ikcp_segment_delete(kcp, seg);
		}
		if (kcp->buffer) {
			ikcp_free(kcp->buffer);
		}
		if (kcp->acklist) {
			ikcp_free(kcp->acklist);
		}

		kcp->nrcv_buf = 0;
		kcp->nsnd_buf = 0;
		kcp->nrcv_que = 0;
		kcp->nsnd_que = 0;
		kcp->ackcount = 0;
		kcp->buffer = NULL;
		kcp->acklist = NULL;
		ikcp_free(kcp);
	}
}


//---------------------------------------------------------------------
// set output callback, which will be invoked by kcp
//---------------------------------------------------------------------
void ikcp_setoutput(ikcpcb *kcp, int (*output)(const char *buf, int len,
	ikcpcb *kcp, void *user))
{
	kcp->output = output;
}


//---------------------------------------------------------------------
// user/upper level recv: returns size, returns below zero for EAGAIN
//---------------------------------------------------------------------
int ikcp_recv(ikcpcb *kcp, char *buffer, int len)
{
	struct IQUEUEHEAD *p;
	int ispeek = (len < 0)? 1 : 0;
	int peeksize;
	int recover = 0;
	IKCPSEG *seg;
	assert(kcp);

	if (iqueue_is_empty(&kcp->rcv_queue))
		return -1;

	if (len < 0) len = -len;

	peeksize = ikcp_peeksize(kcp);

	if (peeksize < 0) 
		return -2;

	if (peeksize > len) 
		return -3;

	if (kcp->nrcv_que >= kcp->rcv_wnd)
		recover = 1;

	// merge fragment
	for (len = 0, p = kcp->rcv_queue.next; p != &kcp->rcv_queue; ) {
		int fragment;
		seg = iqueue_entry(p, IKCPSEG, node);
		p = p->next;

		if (buffer) {
			memcpy(buffer, seg->data, seg->len);
			buffer += seg->len;
		}

		len += seg->len;
		fragment = seg->frg;

		if (ikcp_canlog(kcp, IKCP_LOG_RECV)) {
			ikcp_log(kcp, IKCP_LOG_RECV, "recv sn=%lu", seg->sn);
		}

		if (ispeek == 0) {
			iqueue_del(&seg->node);
			ikcp_segment_delete(kcp, seg);
			kcp->nrcv_que--;
		}

		if (fragment == 0) 
			break;
	}

	assert(len == peeksize);

	// move available data from rcv_buf -> rcv_queue
	while (! iqueue_is_empty(&kcp->rcv_buf)) {
		IKCPSEG *seg = iqueue_entry(kcp->rcv_buf.next, IKCPSEG, node);
		if (seg->sn == kcp->rcv_nxt && kcp->nrcv_que < kcp->rcv_wnd) {
			iqueue_del(&seg->node);
			kcp->nrcv_buf--;
			iqueue_add_tail(&seg->node, &kcp->rcv_queue);
			kcp->nrcv_que++;
			kcp->rcv_nxt++;
		}	else {
			break;
		}
	}

	// fast recover
	if (kcp->nrcv_que < kcp->rcv_wnd && recover) {
		// ready to send back IKCP_CMD_WINS in ikcp_flush
		// tell remote my window size
		kcp->probe |= IKCP_ASK_TELL;
	}

	return len;
}


//---------------------------------------------------------------------
// peek data size
//---------------------------------------------------------------------
int ikcp_peeksize(const ikcpcb *kcp)
{
	struct IQUEUEHEAD *p;
	IKCPSEG *seg;
	int length = 0;

	assert(kcp);

	if (iqueue_is_empty(&kcp->rcv_queue)) return -1;

	seg = iqueue_entry(kcp->rcv_queue.next, IKCPSEG, node);
	if (seg->frg == 0) return seg->len;

	if (kcp->nrcv_que < seg->frg + 1) return -1;

	for (p = kcp->rcv_queue.next; p != &kcp->rcv_queue; p = p->next) {
		seg = iqueue_entry(p, IKCPSEG, node);
		length += seg->len;
		if (seg->frg == 0) break;
	}

	return length;
}

//static void ikcp_dump(const char* prefix, const uint8_t* buf, int size)
//{
//	int i, n;
//	char msg[2048];
//	for (n = i = 0; i < size && i < 100; i++)
//		n += snprintf(msg + n, sizeof(msg) - n, "%u ", (unsigned int)buf[i]);
//	printf("%s: %s\n", prefix, msg);
//}

//---------------------------------------------------------------------
// user/upper level send, returns below zero for error
//---------------------------------------------------------------------
int ikcp_send(ikcpcb *kcp, const char *buffer, int len)
{
	IKCPSEG *seg;
	IUINT32 count, i;

	assert(kcp->mss > 0);
	if (len < 0) return -1;

	kcp_update_bitrate(kcp, len);

	// append to previous segment in streaming mode (if possible)
	if (kcp->stream != 0) {
		if (!iqueue_is_empty(&kcp->snd_queue)) {
			IKCPSEG *old = iqueue_entry(kcp->snd_queue.prev, IKCPSEG, node);
			if (old->len < kcp->mss) {
				int capacity = kcp->mss - old->len;
				int extend = (len < capacity)? len : capacity;
				seg = ikcp_segment_new(kcp, old->len + extend);
				assert(seg);
				if (seg == NULL) {
					return -2;
				}
				iqueue_add_tail(&seg->node, &kcp->snd_queue);
				memcpy(seg->data, old->data, old->len);
				if (buffer) {
					memcpy(seg->data + old->len, buffer, extend);
					buffer += extend;
				}
				seg->len = old->len + extend;
				seg->frg = 0;
				len -= extend;
				iqueue_del_init(&old->node);
				ikcp_segment_delete(kcp, old);
			}
		}
		if (len <= 0) {
			return 0;
		}
	}

	if (len <= (int)kcp->mss) count = 1;
	else count = (len + kcp->mss - 1) / kcp->mss;

	if (count >= IKCP_WND_RCV) return -2;

	if (count == 0) count = 1;

	// fragment
	for (i = 0; i < count; i++) {
		int size = len > (int)kcp->mss ? (int)kcp->mss : len;
		seg = ikcp_segment_new(kcp, size);
		assert(seg);
		if (seg == NULL) {
			return -2;
		}
		if (buffer && len > 0) {
			memcpy(seg->data, buffer, size);
		}
		seg->len = size;
		seg->frg = (kcp->stream == 0)? (count - i - 1) : 0;
		iqueue_init(&seg->node);
		iqueue_add_tail(&seg->node, &kcp->snd_queue);
		kcp->nsnd_que++;
		if (buffer) {
			buffer += size;
		}
		len -= size;
	}

	return 0;
}


//---------------------------------------------------------------------
// parse ack
//---------------------------------------------------------------------
static void ikcp_update_ack(ikcpcb *kcp, IINT32 rtt)
{
	IINT32 rto = 0;
	if (kcp->rx_srtt == 0) {
		kcp->rx_srtt = rtt;
		kcp->rx_rttval = rtt / 2;
	}	else {
		long delta = rtt - kcp->rx_srtt;
		if (delta < 0) delta = -delta;
		kcp->rx_rttval = (3 * kcp->rx_rttval + delta) / 4;
		kcp->rx_srtt = (7 * kcp->rx_srtt + rtt) / 8;
		if (kcp->rx_srtt < 1) kcp->rx_srtt = 1;
	}
	rto = kcp->rx_srtt + _imax_(kcp->interval, 4 * kcp->rx_rttval);
	kcp->rx_rto = _ibound_(kcp->rx_minrto, rto, IKCP_RTO_MAX);

	rtt = minmax_get(&kcp->rtt);
	if (rtt > 0)
	{
		if (kcp->rx_rto > rtt * 2 + IKCP_RTO_MIN)
			kcp->rx_rto = rtt * 2 + IKCP_RTO_MIN;
		if (kcp->rx_rto < (IINT32)(rtt + IKCP_RTO_MIN))
			kcp->rx_rto = rtt + IKCP_RTO_MIN;
	}
	
	netstat_update_rto(&kcp->netstat, kcp->rx_rto);
}

static void ikcp_shrink_buf(ikcpcb *kcp)
{
	struct IQUEUEHEAD *p = kcp->snd_buf.next;
	if (p != &kcp->snd_buf) {
		IKCPSEG *seg = iqueue_entry(p, IKCPSEG, node);
		kcp->snd_una = seg->sn;
	}	else {
		kcp->snd_una = kcp->snd_nxt;
	}
}

static void ikcp_parse_fastack(ikcpcb *kcp, IUINT32 sn, IUINT64 xmit);
static void ikcp_parse_ack(ikcpcb *kcp, IUINT32 sn, IUINT32 ts)
{
	struct IQUEUEHEAD *p, *next;

	if (_itimediff(sn, kcp->snd_una) < 0 || _itimediff(sn, kcp->snd_nxt) >= 0)
		return;

	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = next) {
		IKCPSEG *seg = iqueue_entry(p, IKCPSEG, node);
		next = p->next;
		if (sn == seg->sn) {
			if(seg->csn == kcp->cong_avoid_sn)
				kcplost_acked(&kcp->clost, seg->cxmit);
			if(ts == seg->ts)
				ikcp_parse_fastack(kcp, seg->sn, seg->txsn);
			// rewrite xmit/ts, check band ???
			seg->ts = ts;
			seg->xmit = 1;
			kcp_onack(kcp, seg, kcp->current);

			iqueue_del(p);
			ikcp_segment_delete(kcp, seg);
			kcp->nsnd_buf--;
			break;
		}
		if (_itimediff(sn, seg->sn) < 0) {
			break;
		}
	}
}

static void ikcp_parse_una(ikcpcb *kcp, IUINT32 una)
{
	struct IQUEUEHEAD *p, *next;
	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = next) {
		IKCPSEG *seg = iqueue_entry(p, IKCPSEG, node);
		next = p->next;
		if (_itimediff(una, seg->sn) > 0) {
			if (seg->csn == kcp->cong_avoid_sn)
				kcplost_acked(&kcp->clost, seg->cxmit);
			kcp_onack(kcp, seg, kcp->current);

			iqueue_del(p);
			ikcp_segment_delete(kcp, seg);
			kcp->nsnd_buf--;
		}	else {
			break;
		}
	}
}

static void ikcp_parse_fastack(ikcpcb *kcp, IUINT32 sn, IUINT64 txsn)
{
	struct IQUEUEHEAD *p, *next;

	if (_itimediff(sn, kcp->snd_una) < 0 || _itimediff(sn, kcp->snd_nxt) >= 0)
		return;

	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = next) {
		IKCPSEG *seg = iqueue_entry(p, IKCPSEG, node);
		next = p->next;
		//if (_itimediff(sn, seg->sn) < 0) {
		//	break;
		//} else if (sn != seg->sn) {
		//	seg->fastack++;
		//}

		if (_itimediff(txsn, seg->sn) < 0) {
			break;
		} else if (txsn > seg->txsn) {
			seg->fastack++;
		}
	}
}


//---------------------------------------------------------------------
// ack append
//---------------------------------------------------------------------
static void ikcp_ack_push(ikcpcb *kcp, IUINT32 sn, IUINT32 ts)
{
	size_t newsize = kcp->ackcount + 1;
	IUINT32 *ptr;

	if (newsize > kcp->ackblock) {
		IUINT32 *acklist;
		size_t newblock;

		for (newblock = 8; newblock < newsize; newblock <<= 1);
		acklist = (IUINT32*)ikcp_malloc(newblock * sizeof(IUINT32) * 2);

		if (acklist == NULL) {
			assert(acklist != NULL);
			abort();
		}

		if (kcp->acklist != NULL) {
			size_t x;
			for (x = 0; x < kcp->ackcount; x++) {
				acklist[x * 2 + 0] = kcp->acklist[x * 2 + 0];
				acklist[x * 2 + 1] = kcp->acklist[x * 2 + 1];
			}
			ikcp_free(kcp->acklist);
		}

		kcp->acklist = acklist;
		kcp->ackblock = newblock;
	}

	ptr = &kcp->acklist[kcp->ackcount * 2];
	ptr[0] = sn;
	ptr[1] = ts;
	kcp->ackcount++;
}

static void ikcp_ack_get(const ikcpcb *kcp, int p, IUINT32 *sn, IUINT32 *ts)
{
	if (sn) sn[0] = kcp->acklist[p * 2 + 0];
	if (ts) ts[0] = kcp->acklist[p * 2 + 1];
}


//---------------------------------------------------------------------
// parse data
//---------------------------------------------------------------------
static void ikcp_parse_data(ikcpcb *kcp, IKCPSEG *newseg)
{
	struct IQUEUEHEAD *p, *prev;
	IUINT32 sn = newseg->sn;
	int repeat = 0;
	
	if (_itimediff(sn, kcp->rcv_nxt + kcp->rcv_wnd) >= 0 ||
		_itimediff(sn, kcp->rcv_nxt) < 0) {
		ikcp_segment_delete(kcp, newseg);
		return;
	}

	for (p = kcp->rcv_buf.prev; p != &kcp->rcv_buf; p = prev) {
		IKCPSEG *seg = iqueue_entry(p, IKCPSEG, node);
		prev = p->prev;
		if (seg->sn == sn) {
			repeat = 1;
			break;
		}
		if (_itimediff(sn, seg->sn) > 0) {
			break;
		}
	}

	if (repeat == 0) {
		iqueue_init(&newseg->node);
		iqueue_add(&newseg->node, p);
		kcp->nrcv_buf++;
	}	else {
		ikcp_segment_delete(kcp, newseg);
	}

#if 0
	ikcp_qprint("rcvbuf", &kcp->rcv_buf);
	printf("rcv_nxt=%lu\n", kcp->rcv_nxt);
#endif

	// move available data from rcv_buf -> rcv_queue
	while (! iqueue_is_empty(&kcp->rcv_buf)) {
		IKCPSEG *seg = iqueue_entry(kcp->rcv_buf.next, IKCPSEG, node);
		if (seg->sn == kcp->rcv_nxt && kcp->nrcv_que < kcp->rcv_wnd) {
			iqueue_del(&seg->node);
			kcp->nrcv_buf--;
			iqueue_add_tail(&seg->node, &kcp->rcv_queue);
			kcp->nrcv_que++;
			kcp->rcv_nxt++;
		}	else {
			break;
		}
	}

#if 0
	ikcp_qprint("queue", &kcp->rcv_queue);
	printf("rcv_nxt=%lu\n", kcp->rcv_nxt);
#endif

#if 1
//	printf("snd(buf=%d, queue=%d)\n", kcp->nsnd_buf, kcp->nsnd_que);
//	printf("rcv(buf=%d, queue=%d)\n", kcp->nrcv_buf, kcp->nrcv_que);
#endif
}


//---------------------------------------------------------------------
// input data
//---------------------------------------------------------------------
int ikcp_input(ikcpcb *kcp, const char *data, long size)
{
	IUINT32 una = kcp->snd_una;
	IUINT32 maxack = 0;
	int flag = 0;

	if (ikcp_canlog(kcp, IKCP_LOG_INPUT)) {
		ikcp_log(kcp, IKCP_LOG_INPUT, "[RI] %d bytes", size);
	}

	if (data == NULL || (int)size < (int)IKCP_OVERHEAD) return -1;

	while (1) {
		IUINT32 ts, sn, len, una, conv;
		IUINT16 wnd;
		IUINT8 cmd, frg;
		IKCPSEG *seg;

		if (size < (int)IKCP_OVERHEAD) break;

		data = ikcp_decode32u(data, &conv);
		if (conv != kcp->conv) return -1;

		data = ikcp_decode8u(data, &cmd);
		data = ikcp_decode8u(data, &frg);
		data = ikcp_decode16u(data, &wnd);
		data = ikcp_decode32u(data, &ts);
		data = ikcp_decode32u(data, &sn);
		data = ikcp_decode32u(data, &una);
		data = ikcp_decode32u(data, &len);

		size -= IKCP_OVERHEAD;

		if ((long)size < (long)len) return -2;

		if (cmd != IKCP_CMD_PUSH && cmd != IKCP_CMD_ACK &&
			cmd != IKCP_CMD_WASK && cmd != IKCP_CMD_WINS) 
			return -3;

		kcp->rmt_wnd = wnd;
		ikcp_parse_una(kcp, una);
		ikcp_shrink_buf(kcp);

		if (cmd == IKCP_CMD_ACK) {
			if (_itimediff(kcp->current, ts) >= 0) {
				ikcp_update_ack(kcp, _itimediff(kcp->current, ts));
			}
			ikcp_parse_ack(kcp, sn, ts);
			ikcp_shrink_buf(kcp);
			if (0 == una && sn - una > kcp->snd_wnd)
			{
				// server closed connection
				// 1. sn0(snd_una=0) ---> server: new connection (rcv_nxt=0)
				// 2. sn0(snd_una=1) <--- server ack0 (rcv_nxt=1)
				// 3. snX(snd_una=X+1) <--- server ackX (rcv_nxt=X+1)
				// 4. server close connection
				// 5. snX1(snd_una=X+1) --> server: new connection (rcv_nxt=0)
				// 6. snX1(snd_una=X+2) <-- ackX1 (rcv_nxt=0)
				// 7. snX2(snd_una=X+2) --> server(rcv_nxt=0)
				// 8. snX2(snd_una=X+3) <-- ackX2 (rcv_nxt=0)
				return -102; // ENETRESET
			}

			if (flag == 0) {
				flag = 1;
				maxack = sn;
			}	else {
				if (_itimediff(sn, maxack) > 0) {
					maxack = sn;
				}
			}
			if (ikcp_canlog(kcp, IKCP_LOG_IN_ACK)) {
				ikcp_log(kcp, IKCP_LOG_IN_DATA, 
					"input ack: sn=%lu rtt=%ld rto=%ld", sn, 
					(long)_itimediff(kcp->current, ts),
					(long)kcp->rx_rto);
			}
		}
		else if (cmd == IKCP_CMD_PUSH) {
			if (ikcp_canlog(kcp, IKCP_LOG_IN_DATA)) {
				ikcp_log(kcp, IKCP_LOG_IN_DATA, 
					"input psh: sn=%lu ts=%lu", sn, ts);
			}
			if (_itimediff(sn, kcp->rcv_nxt + kcp->rcv_wnd) < 0) {
				ikcp_ack_push(kcp, sn, ts);
				if (_itimediff(sn, kcp->rcv_nxt) >= 0) {
					seg = ikcp_segment_new(kcp, len);
					seg->conv = conv;
					seg->cmd = cmd;
					seg->frg = frg;
					seg->wnd = wnd;
					seg->ts = ts;
					seg->sn = sn;
					seg->una = una;
					seg->len = len;

					if (len > 0) {
						memcpy(seg->data, data, len);
					}

					ikcp_parse_data(kcp, seg);
				}
			}
		}
		else if (cmd == IKCP_CMD_WASK) {
			// ready to send back IKCP_CMD_WINS in ikcp_flush
			// tell remote my window size
			kcp->probe |= IKCP_ASK_TELL;
			if (ikcp_canlog(kcp, IKCP_LOG_IN_PROBE)) {
				ikcp_log(kcp, IKCP_LOG_IN_PROBE, "input probe");
			}
		}
		else if (cmd == IKCP_CMD_WINS) {
			// do nothing
			if (ikcp_canlog(kcp, IKCP_LOG_IN_WINS)) {
				ikcp_log(kcp, IKCP_LOG_IN_WINS,
					"input wins: %lu", (IUINT32)(wnd));
			}
		}
		else {
			return -3;
		}

		data += len;
		size -= len;
	}

	if (flag != 0) {
		//ikcp_parse_fastack(kcp, maxack);
	}

	if (_itimediff(kcp->snd_una, una) > 0) {
		if (kcp->cwnd < kcp->rmt_wnd) {
			IUINT32 mss = kcp->mss;
			if (kcp->cwnd < kcp->ssthresh) {
				kcp->cwnd++;
				kcp->incr += mss;
			}	else {
				if (kcp->incr < mss) kcp->incr = mss;
				kcp->incr += (mss * mss) / kcp->incr + (mss / 16);
				if ((kcp->cwnd + 1) * mss <= kcp->incr) {
					kcp->cwnd++;
				}
			}
			if (kcp->cwnd > kcp->rmt_wnd) {
				kcp->cwnd = kcp->rmt_wnd;
				kcp->incr = kcp->rmt_wnd * mss;
			}
		}
	}

	//ikcp_trace(kcp, "[%u] input cwnd: %d, ssthresh: %d, rmt_wnd: %d, recv_buf_size: %d, rtt: %d, bitrate: %.1fkbps, bandwidth: %.1fkbps\n", kcp->current, kcp->cwnd, kcp->ssthresh, kcp->rmt_wnd, kcp->nrcv_buf, minmax_get(&kcp->rtt), BANDWIDTH(minmax_get(&kcp->bitrate)), BANDWIDTH(minmax_get(&kcp->bandwidth)));

	return 0;
}


//---------------------------------------------------------------------
// ikcp_encode_seg
//---------------------------------------------------------------------
static char *ikcp_encode_seg(char *ptr, const IKCPSEG *seg)
{
	ptr = ikcp_encode32u(ptr, seg->conv);
	ptr = ikcp_encode8u(ptr, (IUINT8)seg->cmd);
	ptr = ikcp_encode8u(ptr, (IUINT8)seg->frg);
	ptr = ikcp_encode16u(ptr, (IUINT16)seg->wnd);
	ptr = ikcp_encode32u(ptr, seg->ts);
	ptr = ikcp_encode32u(ptr, seg->sn);
	ptr = ikcp_encode32u(ptr, seg->una);
	ptr = ikcp_encode32u(ptr, seg->len);
	return ptr;
}

static int ikcp_wnd_unused(const ikcpcb *kcp)
{
	if (kcp->nrcv_que < kcp->rcv_wnd) {
		return kcp->rcv_wnd - kcp->nrcv_que;
	}
	return 0;
}


static char* ikcp_flush_segment(ikcpcb* kcp, IUINT32 current, char *ptr)
{
	int size, need;
	IUINT32 cwnd, rtomin;
	char *buffer = kcp->buffer;

	// calculate window size
	cwnd = _imin_(kcp->snd_wnd, kcp->rmt_wnd);
	if (kcp->nocwnd == 0) cwnd = _imin_(kcp->cwnd, cwnd);
	
	if (0 == cwnd || kcp->snd_una + cwnd <= kcp->snd_nxt)
	{
		kcp->total_bytes = kcp->total_send + kcp->mtu; // reset bdp
		//kcp_notify(kcp, "[%u] bdp %u, cwnd: %u, rto: %d, rtt: %u, rwnd: %u, total: %"PRIu64", send: %"PRIu64"", current, kcp->snd_nxt, cwnd, kcp->rx_rto, minmax_get(&kcp->rtt), kcp->rmt_wnd, kcp->total_bytes, kcp->total_send);
	}

	rtomin = (kcp->nodelay == 0) ? (kcp->rx_rto >> 3) : 0;

	// move data from snd_queue to snd_buf
	while (_itimediff(kcp->snd_nxt, kcp->snd_una + cwnd) < 0) {
		IKCPSEG *newseg;
		if (iqueue_is_empty(&kcp->snd_queue)) break;

		newseg = iqueue_entry(kcp->snd_queue.next, IKCPSEG, node);

		if (0 == kcp_pacing(kcp, newseg))
			break;

		iqueue_del(&newseg->node);
		iqueue_add_tail(&newseg->node, &kcp->snd_buf);
		kcp->nsnd_que--;
		kcp->nsnd_buf++;

		newseg->conv = kcp->conv;
		newseg->cmd = IKCP_CMD_PUSH;
		newseg->wnd = ikcp_wnd_unused(kcp);
		newseg->ts = current;
		newseg->sn = kcp->snd_nxt++;
		newseg->una = kcp->rcv_nxt;
		newseg->rto = kcp->rx_rto;
		newseg->resendts = current + newseg->rto + rtomin;
		newseg->fastack = 0;
		newseg->xmit = 1;

		newseg->csn = kcp->cong_avoid_sn;
		newseg->cxmit = 1;
		newseg->txsn = kcp->txsn++;
		newseg->delivered = kcp->delivered;
		//kcplost_send(&kcp->clost, minmax_get(&kcp->rtt), current);
		kcplost_send(&kcp->clost, newseg->cxmit);
		kcp->netstat.count++;
		ikcp_trace(kcp, "[%u] send %u init, snd_nxt: %u, snd_buf: %d, snd_queue: %d, snd_uack: %d(%d), rto: %d, snd_txsn: %"PRIu64", total: %"PRIu64", send: %"PRIu64"\n", current, newseg->sn, kcp->snd_nxt, kcp->nsnd_buf, kcp->nsnd_que, kcp->snd_una, kcp->snd_nxt - kcp->snd_una, kcp->rx_rto, kcp->txsn, kcp->total_bytes, kcp->total_send);

		// send
		size = (int)(ptr - buffer);
		need = IKCP_OVERHEAD + newseg->len;

		if (size + need > (int)kcp->mtu) {
			ikcp_output(kcp, buffer, size);
			ptr = buffer;
		}

		ptr = ikcp_encode_seg(ptr, newseg);

		if (newseg->len > 0) {
			memcpy(ptr, newseg->data, newseg->len);
			ptr += newseg->len;
		}
	}

	// reset bdp
	if (iqueue_is_empty(&kcp->snd_queue) && kcp->total_send + kcp->mtu < kcp->total_bytes)
	{
		kcp->total_bytes = kcp->total_send + kcp->mtu;
		//ikcp_trace(kcp, "[%u] bdp %u, cwnd: %u, rto: %d, rtt: %u, rwnd: %u, total: %"PRIu64", send: %"PRIu64"\n", current, kcp->snd_nxt, cwnd, kcp->rx_rto, minmax_get(&kcp->rtt), kcp->rmt_wnd, kcp->total_bytes, kcp->total_send);
	}

	return ptr;
}

//---------------------------------------------------------------------
// ikcp_flush
//---------------------------------------------------------------------
void ikcp_flush(ikcpcb *kcp)
{
	IUINT32 current = kcp->current;
	char *buffer = kcp->buffer;
	char *ptr = buffer;
	int count, size, i;
	IUINT32 resent, cwnd;
	IUINT32 rtomin;
	struct IQUEUEHEAD *p;
	int change = 0;
	int lost = 0;
	IKCPSEG seg;

	// 'ikcp_update' haven't been called. 
	if (kcp->updated == 0) return;

	seg.conv = kcp->conv;
	seg.cmd = IKCP_CMD_ACK;
	seg.frg = 0;
	seg.wnd = ikcp_wnd_unused(kcp);
	seg.una = kcp->rcv_nxt;
	seg.len = 0;
	seg.sn = 0;
	seg.ts = 0;

	// flush acknowledges
	count = kcp->ackcount;
	for (i = 0; i < count; i++) {
		size = (int)(ptr - buffer);
		if (size + (int)IKCP_OVERHEAD > (int)kcp->mtu) {
			ikcp_output(kcp, buffer, size);
			ptr = buffer;
		}
		ikcp_ack_get(kcp, i, &seg.sn, &seg.ts);
		ptr = ikcp_encode_seg(ptr, &seg);
	}

	kcp->ackcount = 0;

	// probe window size (if remote window size equals zero)
	if (kcp->rmt_wnd == 0) {
		if (kcp->probe_wait == 0) {
			kcp->probe_wait = IKCP_PROBE_INIT;
			kcp->ts_probe = kcp->current + kcp->probe_wait;
		}	
		else {
			if (_itimediff(kcp->current, kcp->ts_probe) >= 0) {
				if (kcp->probe_wait < IKCP_PROBE_INIT) 
					kcp->probe_wait = IKCP_PROBE_INIT;
				kcp->probe_wait += kcp->probe_wait / 2;
				if (kcp->probe_wait > IKCP_PROBE_LIMIT)
					kcp->probe_wait = IKCP_PROBE_LIMIT;
				kcp->ts_probe = kcp->current + kcp->probe_wait;
				kcp->probe |= IKCP_ASK_SEND;
			}
		}
	}	else {
		kcp->ts_probe = 0;
		kcp->probe_wait = 0;
	}

	// flush window probing commands
	if (kcp->probe & IKCP_ASK_SEND) {
		seg.cmd = IKCP_CMD_WASK;
		size = (int)(ptr - buffer);
		if (size + (int)IKCP_OVERHEAD > (int)kcp->mtu) {
			ikcp_output(kcp, buffer, size);
			ptr = buffer;
		}
		ptr = ikcp_encode_seg(ptr, &seg);
	}

	// flush window probing commands
	if (kcp->probe & IKCP_ASK_TELL) {
		seg.cmd = IKCP_CMD_WINS;
		size = (int)(ptr - buffer);
		if (size + (int)IKCP_OVERHEAD > (int)kcp->mtu) {
			ikcp_output(kcp, buffer, size);
			ptr = buffer;
		}
		ptr = ikcp_encode_seg(ptr, &seg);
	}

	kcp->probe = 0;

	//inflight = kcp_get_inflight(kcp);
	//bdp = kcp_get_bdp(kcp);

	//// calculate window size
	cwnd = _imin_(kcp->snd_wnd, kcp->rmt_wnd);
	if (kcp->nocwnd == 0) cwnd = _imin_(kcp->cwnd, cwnd);

	//// move data from snd_queue to snd_buf
	//while (_itimediff(kcp->snd_nxt, kcp->snd_una + cwnd) < 0) {
	//	IKCPSEG *newseg;
	//	if (iqueue_is_empty(&kcp->snd_queue)) break;

	//	newseg = iqueue_entry(kcp->snd_queue.next, IKCPSEG, node);

	//	if (inflight >= bdp) {
	//		ikcp_trace(kcp, "[%u] bdp %d, inflight: %d, unsend: %d, segment: %d, slowstart: %d\n", kcp->current, bdp, inflight, kcp_get_unsend(kcp), kcp->snd_nxt, kcp->slowstart);
	//		break;
	//	}
	//	inflight += newseg->len + IKCP_OVERHEAD + 8 /*UDP*/ + 20 /*IP*/;

	//	iqueue_del(&newseg->node);
	//	iqueue_add_tail(&newseg->node, &kcp->snd_buf);
	//	kcp->nsnd_que--;
	//	kcp->nsnd_buf++;

	//	newseg->conv = kcp->conv;
	//	newseg->cmd = IKCP_CMD_PUSH;
	//	newseg->wnd = seg.wnd;
	//	newseg->ts = current;
	//	newseg->sn = kcp->snd_nxt++;
	//	newseg->una = kcp->rcv_nxt;
	//	newseg->resendts = current;
	//	newseg->rto = kcp->rx_rto;
	//	newseg->fastack = 0;
	//	newseg->xmit = 0;

	//	newseg->delivered = kcp->delivered;
	//}

	kcp_congest_avoid(kcp, kcp->current);
	kcp_update_threshold(kcp, kcp->current);

	// calculate resent
	resent = (kcp->fastresend > 0)? (IUINT32)kcp->fastresend : 0xffffffff;
	rtomin = (kcp->nodelay == 0)? (kcp->rx_rto >> 3) : 0;

	// flush data segments
	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = p->next) {
		IKCPSEG *segment = iqueue_entry(p, IKCPSEG, node);
		int needsend = 0;
		if (segment->xmit == 0) {
			if (0 == kcp_pacing(kcp, segment))
				break;

			needsend = 1;
			segment->xmit++;
			segment->rto = kcp->rx_rto;
			segment->resendts = current + segment->rto + rtomin;
			ikcp_trace(kcp, "[%u] send %u init, snd_nxt: %u, snd_buf: %d, snd_queue: %d, snd_uack: %d(%d), rto: %d, snd_txsn: %"PRIu64", total: %"PRIu64", send: %"PRIu64"\n", current, segment->sn, kcp->snd_nxt, kcp->nsnd_buf, kcp->nsnd_que, kcp->snd_una, kcp->snd_nxt - kcp->snd_una, kcp->rx_rto, kcp->txsn, kcp->total_bytes, kcp->total_send);
		}
		else if (__gt__(current, segment->resendts)) {
			lost++; // mark as timeout
			if (segment->xmit == 1 && segment->fastack < 1 && segment->sn + 1 < kcp->snd_nxt /*&& __gt__(segment->resendts + KCP_TIMEOUT_DELAY, current)*/ )
			{
				// do nothing
			}
			else
			{
				if (0 == kcp_pacing(kcp, segment))
					break;

				needsend = 1;
				segment->xmit++;
				kcp->netstat.lost++;
				kcp->xmit++;

				//if (kcp->nodelay == 0) {
				//	segment->rto += kcp->rx_rto;
				//} else {
				//	segment->rto += kcp->rx_rto / 2;
				//}
				segment->resendts = current + segment->rto;
				ikcp_trace(kcp, "[%u] send %u timeout, snd_nxt: %u, snd_buf: %d, snd_queue: %d, snd_uack: %d(%d), rto: %d, snd_txsn: %"PRIu64", total: %"PRIu64", send: %"PRIu64"\n", current, segment->sn, kcp->snd_nxt, kcp->nsnd_buf, kcp->nsnd_que, kcp->snd_una, kcp->snd_nxt - kcp->snd_una, kcp->rx_rto, kcp->txsn, kcp->total_bytes, kcp->total_send);
			}
		}
		else if (segment->fastack >= resent && segment->xmit < KCP_FAST_RETRANSMISSION) {
			if (0 == kcp_pacing(kcp, segment))
				break;

			//kcpcongest_update(&kcp->crtt, current - segment->ts, current);
			needsend = 1;
			kcp->netstat.fastack++;
			segment->xmit++;
			segment->fastack = 0;
			segment->resendts = current + segment->rto;
			change++;

			ikcp_trace(kcp, "[%u] send %u fast, snd_nxt: %u, snd_buf: %d, snd_queue: %d, snd_uack: %d(%d), snd_txsn: %"PRIu64", rto: %d, total: %"PRIu64", send: %"PRIu64"\n", current, segment->sn, kcp->snd_nxt, kcp->nsnd_buf, kcp->nsnd_que, kcp->snd_una, kcp->snd_nxt - kcp->snd_una, segment->txsn, kcp->rx_rto, kcp->total_bytes, kcp->total_send);
		}

		if (needsend) {
			int size, need;
			segment->ts = current;
			segment->wnd = seg.wnd;
			segment->una = kcp->rcv_nxt;
			segment->fastack = 0;
			segment->txsn = kcp->txsn++;
			segment->delivered = kcp->delivered;
			if (segment->xmit >= KCP_FAST_RETRANSMISSION && segment->rto > KCP_TIMEOUT_THRESHOLD /*ms*/)
				segment->resendts = current + KCP_TIMEOUT_THRESHOLD; // fast retransmission
			kcp->netstat.count++;
			if (segment->csn != kcp->cong_avoid_sn) {
				segment->csn = kcp->cong_avoid_sn;
				segment->cxmit = 1;
			} else {
				segment->cxmit++;
			}
			//kcplost_send(&kcp->clost, minmax_get(&kcp->rtt), current);
			kcplost_send(&kcp->clost, segment->cxmit);
			
			size = (int)(ptr - buffer);
			need = IKCP_OVERHEAD + segment->len;

			if (size + need > (int)kcp->mtu) {
				ikcp_output(kcp, buffer, size);
				ptr = buffer;
			}

			ptr = ikcp_encode_seg(ptr, segment);

			if (segment->len > 0) {
				memcpy(ptr, segment->data, segment->len);
				ptr += segment->len;
			}

			if (segment->xmit >= kcp->dead_link) {
				kcp->state = -1;
			}
		}
	}

	ptr = ikcp_flush_segment(kcp, kcp->current, ptr);

	// flash remain segments
	size = (int)(ptr - buffer);
	if (size > 0) {
		ikcp_output(kcp, buffer, size);
	}

	// update ssthresh
	if (change) {
		IUINT32 inflight = kcp->snd_nxt - kcp->snd_una;
		kcp->ssthresh = inflight / 2;
		if (kcp->ssthresh < IKCP_THRESH_MIN)
			kcp->ssthresh = IKCP_THRESH_MIN;
		kcp->cwnd = kcp->ssthresh + resent;
		kcp->incr = kcp->cwnd * kcp->mss;
	}

	if (lost) {
		kcp->ssthresh = cwnd / 2;
		if (kcp->ssthresh < IKCP_THRESH_MIN)
			kcp->ssthresh = IKCP_THRESH_MIN;
		kcp->cwnd = 1;
		kcp->incr = kcp->mss;
	}

	if (kcp->cwnd < 1) {
		kcp->cwnd = 1;
		kcp->incr = kcp->mss;
	}
}


//---------------------------------------------------------------------
// update state (call it repeatedly, every 10ms-100ms), or you can ask 
// ikcp_check when to call it again (without ikcp_input/_send calling).
// 'current' - current timestamp in millisec. 
//---------------------------------------------------------------------
void ikcp_update(ikcpcb *kcp, IUINT32 current)
{
	IINT32 slap;

	kcp->current = current;

	if (kcp->updated == 0) {
		kcp->updated = 1;
		kcp->ts_flush = kcp->current;
	}

	slap = _itimediff(kcp->current, kcp->ts_flush);

	if (slap >= 10000 || slap < -10000) {
		kcp->ts_flush = kcp->current;
		slap = 0;
	}

	if (slap >= 0) {
		kcp->ts_flush += kcp->interval;
		if (_itimediff(kcp->current, kcp->ts_flush) >= 0) {
			kcp->ts_flush = kcp->current + kcp->interval;
		}
		ikcp_flush(kcp);
	}
}


//---------------------------------------------------------------------
// Determine when should you invoke ikcp_update:
// returns when you should invoke ikcp_update in millisec, if there 
// is no ikcp_input/_send calling. you can call ikcp_update in that
// time, instead of call update repeatly.
// Important to reduce unnacessary ikcp_update invoking. use it to 
// schedule ikcp_update (eg. implementing an epoll-like mechanism, 
// or optimize ikcp_update when handling massive kcp connections)
//---------------------------------------------------------------------
IUINT32 ikcp_check(const ikcpcb *kcp, IUINT32 current)
{
	IUINT32 ts_flush = kcp->ts_flush;
	IINT32 tm_flush = 0x7fffffff;
	IINT32 tm_packet = 0x7fffffff;
	IUINT32 minimal = 0;
	struct IQUEUEHEAD *p;

	if (kcp->updated == 0) {
		return current;
	}

	if (_itimediff(current, ts_flush) >= 10000 ||
		_itimediff(current, ts_flush) < -10000) {
		ts_flush = current;
	}

	if (_itimediff(current, ts_flush) >= 0) {
		return current;
	}

	tm_flush = _itimediff(ts_flush, current);

	for (p = kcp->snd_buf.next; p != &kcp->snd_buf; p = p->next) {
		const IKCPSEG *seg = iqueue_entry(p, const IKCPSEG, node);
		IINT32 diff = _itimediff(seg->resendts, current);
		if (diff <= 0) {
			return current;
		}
		if (diff < tm_packet) tm_packet = diff;
	}

	minimal = (IUINT32)(tm_packet < tm_flush ? tm_packet : tm_flush);
	if (minimal >= kcp->interval) minimal = kcp->interval;

	return current + minimal;
}



int ikcp_setmtu(ikcpcb *kcp, int mtu)
{
	char *buffer;
	if (mtu < 50 || mtu < (int)IKCP_OVERHEAD) 
		return -1;
	buffer = (char*)ikcp_malloc((mtu + IKCP_OVERHEAD) * 3);
	if (buffer == NULL) 
		return -2;
	kcp->mtu = mtu;
	kcp->mss = kcp->mtu - IKCP_OVERHEAD;
	ikcp_free(kcp->buffer);
	kcp->buffer = buffer;
	return 0;
}

static int ikcp_interval(ikcpcb *kcp, int interval)
{
	if (interval > 5000) interval = 5000;
	else if (interval < 10) interval = 10;
	kcp->interval = interval;
	return 0;
}

int ikcp_nodelay(ikcpcb *kcp, int nodelay, int interval, int resend, int nc)
{
	if (nodelay >= 0) {
		kcp->nodelay = nodelay;
		if (nodelay) {
			kcp->rx_minrto = IKCP_RTO_NDL;	
		}	
		else {
			kcp->rx_minrto = IKCP_RTO_MIN;
		}
	}
	if (interval >= 0) {
		if (interval > KCP_INTERVAL_MAX) interval = KCP_INTERVAL_MAX;
		else if (interval < KCP_INTERVAL_MIN) interval = KCP_INTERVAL_MIN;
		kcp->interval = interval;
	}
	if (resend >= 0) {
		kcp->fastresend = resend;
	}
	if (nc >= 0) {
		kcp->nocwnd = nc;
	}
	return 0;
}

int ikcp_congestion(ikcpcb *kcp, uint32_t bandwidth, uint32_t lost, double incr, double decr)
{
	kcp->cong_band = bandwidth;
	kcp->cong_lost = lost;
	kcp->cong_incr = incr;
	kcp->cong_decr = decr;
	return 0;
}

int ikcp_wndsize(ikcpcb *kcp, int sndwnd, int rcvwnd)
{
	if (kcp) {
		if (sndwnd > 0) {
			kcp->snd_wnd = sndwnd;
		}
		if (rcvwnd > 0) {   // must >= max fragment size
			kcp->rcv_wnd = _imax_(rcvwnd, IKCP_WND_RCV);
		}
	}
	return 0;
}

int ikcp_waitsnd(const ikcpcb *kcp)
{
	return kcp->nsnd_buf + kcp->nsnd_que;
}


// read conv
IUINT32 ikcp_getconv(const void *ptr)
{
	IUINT32 conv;
	ikcp_decode32u((const char*)ptr, &conv);
	return conv;
}


