//
//  LiteLibrtmpk.c
//  Unionlibrtmpk
//
//

#include "kcp.h"
#include "LiteLibrtmpk.h"
#include "LitePublisherUtils.h"
#include "LitePublisherDef.h"

#include "sys/locker.h"
#include "flv-proto.h"
#include "flv-muxer.h"
#include "amf0.h"
#include "rtmp-client.h"
#include "rtmp-url.h"
#include "ip-route.h"
#include <assert.h>

#define RTMP_METADATA_MAXLEN    1024

struct rtmp_transport_t
{
	int (*open)(struct KCPContext* s, const char *host, int port);
	int (*close)(struct KCPContext *s);

	int (*read)(struct KCPContext *s, uint8_t *buf, int size, int nonblock);
	int (*write)(struct KCPContext *s, const uint8_t *buf, int size);
};

static int tcp_open(struct KCPContext* tcp, const char *host, int port)
{
	struct sockaddr_storage addr;
	socklen_t addrlen = sizeof(addr);

	tcp->sock = socket_invalid;

	uint64_t clock = system_clock();
	int r = socket_addr_from(&addr, &addrlen, host, port);
	tcp->dnsTime = system_clock() - clock;
	if (0 != r)
		return r;
	
	tcp->sock = socket_tcp();
	r = socket_connect_by_time(tcp->sock, (struct sockaddr*)&addr, addrlen, tcp->timeout);
	if (0 != r)
	{
		socket_close(tcp->sock);
		tcp->sock = socket_invalid;
	}

	return r;
}

static int tcp_close(struct KCPContext* tcp)
{
	return socket_close(tcp->sock);
}

static int tcp_read(struct KCPContext* tcp, uint8_t *buf, int size, int nonblock)
{
	return socket_recv_by_time(tcp->sock, buf, size, 0, nonblock ? 0 : tcp->timeout);
}

static int tcp_write(struct KCPContext* tcp, const uint8_t *buf, int size)
{
	if (size != socket_send_all_by_time(tcp->sock, buf, size, 0, tcp->timeout))
		return -ETIMEDOUT;
	return 0;
}

/**
 * UnionLibrtmpk对象
 */
typedef struct LiteLibrtmpk {
	struct rtmp_transport_t transport;
	struct KCPContext		kcp;
	struct rtmp_url_t		url;
	struct flv_muxer_t		*flvMuxer;
	struct rtmp_client_t	*rtmpHandle;

	char					local[128];
	char					remote[128];
	uint8_t					*audioBuffer;
	int						audioCapacity;

	bool                    bInitAudio;
	bool                    bSendMeta;

	int64_t                 videoBaseTime;
	int64_t                 audioBaseTime;

	int64_t                 lastVideoTime;
	int64_t                 lastAudioTime;

	UnionPublisherStatus    status;

	UnionVideoEncCfg        videoEncCfg;
	UnionAudioEncCfg        audioEncCfg;

	locker_t				mutex;
	locker_t				destroyMutex;

	UnionDict               userMetadata;
}LiteLibrtmpk_t;

static int rtmp_client_send(void* param, const void* header, size_t len, const void* data, size_t bytes);

/**
 * @abstract 转换时间戳
 */
static uint32_t union_librtmpk_get_relativeTime(LiteLibrtmpk_t *librtmpk, int64_t timestamp, UnionMediaType type)
{
    uint32_t ret = 0;
    
    if(NULL == librtmpk)
        return 0;

    if(UNION_MEDIA_TYPE_AUDIO == type)
    {
        if(librtmpk->audioBaseTime < 0)
        {
            librtmpk->audioBaseTime = timestamp;
            librtmpk->lastAudioTime = timestamp;
        }
        
        if(timestamp < librtmpk->lastAudioTime)
        {
//            UnionLogE("ERROR!!!! audio timestamp %lld is smaller than last one : %lld\n", timestamp, librtmpk->lastAudioTime);
            timestamp = librtmpk->lastAudioTime;
        }
        else
            librtmpk->lastAudioTime = timestamp;
        
        ret = timestamp - librtmpk->audioBaseTime;
    }
    else if(UNION_MEDIA_TYPE_VIDEO == type)
    {
        if(librtmpk->videoBaseTime < 0)
        {
            librtmpk->videoBaseTime = timestamp;
            librtmpk->lastVideoTime = timestamp;
        }
        
        if(timestamp < librtmpk->lastVideoTime)
        {
//            UnionLogE("ERROR!!!! video timestamp %lld is smaller than last one : %lld\n", timestamp, librtmpk->lastVideoTime);
            timestamp = librtmpk->lastVideoTime;
        }
        else
            librtmpk->lastVideoTime = timestamp;
        
        ret = timestamp - librtmpk->videoBaseTime;
    }

    return ret;
}

static int union_librtmpk_send_packet(void* param, int type, const void* data, size_t bytes, uint32_t timestamp)
{
	LiteLibrtmpk_t *librtmpk = (LiteLibrtmpk_t*)param;
	switch (type)
	{
	case FLV_TYPE_AUDIO:
		return rtmp_client_push_audio(librtmpk->rtmpHandle, data, bytes, timestamp);
	case FLV_TYPE_VIDEO:
		return rtmp_client_push_video(librtmpk->rtmpHandle, data, bytes, timestamp);
	case FLV_TYPE_SCRIPT:
		return rtmp_client_push_script(librtmpk->rtmpHandle, data, bytes, timestamp);
	default:
		return -1;
	}
}

/**
 * @abstract 发送视频帧
 */
static int union_librtmpk_send_videoframe(LiteLibrtmpk_t *librtmpk, UnionAVPacket *packet)
{
    int ret = -1;
    
    if( NULL == packet || NULL == packet->data)
        return -1;
    
	uint32_t timestamp = union_librtmpk_get_relativeTime(librtmpk, packet->dts, packet->type);
	if (UNION_CODEC_ID_H264 == librtmpk->videoEncCfg.codecId)
	{
		return flv_muxer_avc(librtmpk->flvMuxer, packet->data, packet->size, (int)(packet->pts - packet->dts) + timestamp, timestamp);
	}
	else if (UNION_CODEC_ID_H265 == librtmpk->videoEncCfg.codecId)
	{
		return flv_muxer_hevc(librtmpk->flvMuxer, packet->data, packet->size, (int)(packet->pts - packet->dts) + timestamp, timestamp);
	}
	else
	{
		assert(0);
		return -1;
	}

    return ret;
}

/**
 * @abstract 发送音频帧
 */
static int union_librtmpk_send_audioframe(LiteLibrtmpk_t *librtmpk, UnionAVPacket *packet)
{
    int ret = -1;
    
    if(NULL == packet || NULL == packet->data)
        return -1;

	if (UNION_CODEC_ID_AAC != librtmpk->audioEncCfg.codecId)
		return -1;

	uint32_t timestamp = union_librtmpk_get_relativeTime(librtmpk, packet->dts, packet->type);
#if defined(_DEBUG) || defined(DEBUG)
	if (packet->size > 7 && 0xFF == packet->data[0] && 0xF0 == (packet->data[1] & 0xF0))
	{
		return flv_muxer_aac(librtmpk->flvMuxer, packet->data, packet->size, (packet->pts - packet->dts) + timestamp, timestamp);
	}
#endif

	if (2 + packet->size > librtmpk->audioCapacity)
	{
		void *ptr = realloc(librtmpk->audioBuffer, packet->size + 16 * 1024);
		if (NULL == ptr)
			return -1;
		librtmpk->audioBuffer = (uint8_t*)ptr;
		librtmpk->audioCapacity = packet->size + 16 * 1024;
	}
    
	//uint32_t timestamp = union_librtmpk_get_relativeTime(librtmpk, packet->dts, packet->type);
	if (packet->flags & UNION_AV_FLAG_CODEC_CONFIG)
    {
		librtmpk->audioBuffer[0] = (FLV_AUDIO_AAC /*<< 4*/) /* SoundFormat */ | (3 << 2) /* 44k-SoundRate */ | (1 << 1) /* 16-bit samples */ | 1 /* Stereo sound */;
		librtmpk->audioBuffer[1] = 0x00;
		memcpy(librtmpk->audioBuffer + 2, packet->data, packet->size);
		ret = rtmp_client_push_audio(librtmpk->rtmpHandle, librtmpk->audioBuffer, packet->size + 2, timestamp);
        if(ret >= 0)
            librtmpk->bInitAudio = true;
    }
    else
    {
        if(!librtmpk->bInitAudio)
            return -1;
        
		librtmpk->audioBuffer[0] = (FLV_AUDIO_AAC /*<< 4*/) /* SoundFormat */ | (3 << 2) /* 44k-SoundRate */ | (1 << 1) /* 16-bit samples */ | 1 /* Stereo sound */;
		librtmpk->audioBuffer[1] = 0x01;
		memcpy(librtmpk->audioBuffer + 2, packet->data, packet->size);
		ret = rtmp_client_push_audio(librtmpk->rtmpHandle, librtmpk->audioBuffer, packet->size + 2, timestamp);
    }
    
    return ret;
}

/**
 * @abstract 释放user-define Metadata中的所有元素
 */
static void union_librtmpk_free_dict(UnionDict *dict)
{
    if(NULL == dict || dict->number == 0)
        return ;
    
    for(int i = 0; i < dict->number; i++)
    {
        UnionDictElem *pElem = &dict->elems[i];
        if(pElem->name)
        {
            free(pElem->name);
            pElem->name = NULL;
        }
        
        if(UnionDataType_String == pElem->type)
        {
            free(pElem->val.string);
            pElem->val.string = NULL;
        }
    }
    
    free(dict->elems);
    dict->elems = NULL;
    dict->number = 0;
}

/**
 * @abstract 组装metada
 */
static int union_librtmpk_compose_metadata(LiteLibrtmpk_t *librtmpk, uint8_t *buffer, int size)
{
    UnionVideoEncCfg *videoEncCfg = NULL;
    UnionAudioEncCfg *audioEncCfg = NULL;
    uint8_t *out, *end = NULL;
    
    if(NULL == buffer || size <= 0)
        return -1;
    
	out = buffer;
	end = out + size;
	out = AMFWriteString(out, end, "@setDataFrame", 13);
	out = AMFWriteString(out, end, "onMetaData", 10);
	out = AMFWriteECMAArarry(out, end);

	//video
	videoEncCfg = &librtmpk->videoEncCfg;
	out = AMFWriteNamedDouble(out, end, "duration", 8, 0);
	out = AMFWriteNamedDouble(out, end, "width", 5, videoEncCfg->width);
	out = AMFWriteNamedDouble(out, end, "height", 6, videoEncCfg->height);
	out = AMFWriteNamedDouble(out, end, "framerate", 9, videoEncCfg->frameRate);
	out = AMFWriteNamedDouble(out, end, "videodatarate", 13, videoEncCfg->bitrate/1024.0);
	out = AMFWriteNamedDouble(out, end, "interval", 8, videoEncCfg->iFrameInterval);
	if (UNION_CODEC_ID_H264 == videoEncCfg->codecId)
		out = AMFWriteNamedDouble(out, end, "videocodecid", 12, FLV_VIDEO_H264);
	
	//audio
	audioEncCfg = &librtmpk->audioEncCfg;
	out = AMFWriteNamedDouble(out, end, "audiodatarate", 13, audioEncCfg->bitrate/1024.0);
	out = AMFWriteNamedDouble(out, end, "audiosamplerate", 15, audioEncCfg->sampleRate);
	out = AMFWriteNamedBoolean(out, end, "stereo", 16, audioEncCfg->channels == 2 ? 1 : 0);
	out = AMFWriteNamedDouble(out, end, "audiosamplesize", 15, audioEncCfg->sampleFmt == UNION_SAMPLE_FMT_U8 ? 8 : 16);
	if (UNION_CODEC_ID_AAC == audioEncCfg->codecId)
		out = AMFWriteNamedDouble(out, end, "audiocodecid", 12, 10);

	//user define
	UnionDictElem *dictElem = NULL;
	for (int i = 0; i < librtmpk->userMetadata.number; i++)
	{
		dictElem = &(librtmpk->userMetadata.elems[i]);
		const char* name = (char *)dictElem->name;
		if (UnionDataType_Number == dictElem->type)
			out = AMFWriteNamedDouble(out, end, name, strlen(name), dictElem->val.number);
		else if (UnionDataType_String == dictElem->type)
			out = AMFWriteNamedString(out, end, name, strlen(name), dictElem->val.string, strlen(dictElem->val.string));
	}

	out = AMFWriteObjectEnd(out, end);
	return out - buffer;
}

/**
 * @abstract 发送metada
 */
static void union_librtmpk_send_metadata(LiteLibrtmpk_t *librtmpk)
{
	uint8_t metadata[RTMP_METADATA_MAXLEN];
	rtmp_client_t* rtmp = (rtmp_client_t*)librtmpk->rtmpHandle;
    
	int size = union_librtmpk_compose_metadata(librtmpk, metadata, RTMP_METADATA_MAXLEN);
	if (size > 0)
		rtmp_client_push_script(rtmp, metadata, size, 0);
}

/**
 * @abstract 设置默认视频格式
 */
static void union_librtmpk_set_default_videocfg(LiteLibrtmpk_t *librtmpk)
{
    if(NULL == librtmpk)
        return ;
    
    UnionVideoEncCfg *videoEncCfg = &librtmpk->videoEncCfg;
    memset(videoEncCfg, 0, sizeof(UnionVideoEncCfg));
    videoEncCfg->codecId = UNION_CODEC_ID_H264;
    
    return ;
}

/**
 * @abstract 设置默认音频格式
 */
static void union_librtmpk_set_default_audiocfg(LiteLibrtmpk_t *librtmpk)
{
    if(NULL == librtmpk)
        return ;
    
    UnionAudioEncCfg *audioEncCfg = &librtmpk->audioEncCfg;
    memset(audioEncCfg, 0, sizeof(UnionAudioEncCfg));
    audioEncCfg->codecId       = UNION_CODEC_ID_AAC;
    audioEncCfg->profile       = UNION_CODEC_PROFILE_AAC_LOW;
    audioEncCfg->sampleFmt     = UNION_SAMPLE_FMT_S16;
    audioEncCfg->sampleRate    = 44100;
    audioEncCfg->channels      = 1;
    audioEncCfg->bitrate       = 0;
    
    return ;
}

/**
 @abstract 获取推流器当前状态
 
 @param publisher 推流对象
 
 @return 返回推流器当前状态
 */
UnionPublisherStatus union_librtmpk_get_status(LiteLibrtmpk_t *publisher)
{
    if(NULL == publisher)
        return (UnionPublisherStatus)(-1);
    
    return publisher->status;
}

/**
 @abstract 设置当前流的视频格式
 
 @param publisher 推流对象
 */
void union_librtmpk_set_videocfg(LiteLibrtmpk_t *librtmpk, UnionVideoEncCfg *vEncCfg)
{
    if(NULL == librtmpk || NULL == vEncCfg)
        return ;
    
    locker_lock(&librtmpk->mutex);
    memcpy(&(librtmpk->videoEncCfg), vEncCfg, sizeof(UnionVideoEncCfg));
    librtmpk->bSendMeta = false;
    locker_unlock(&librtmpk->mutex);
    
    return ;
}

/**
 @abstract 设置当前流的音频格式
 
 @param publisher 推流对象
 */
void union_librtmpk_set_audiocfg(LiteLibrtmpk_t *librtmpk, UnionAudioEncCfg *aEncCfg)
{
    if(NULL == librtmpk || NULL == aEncCfg)
        return ;
    
    locker_lock(&librtmpk->mutex);
    memcpy(&(librtmpk->audioEncCfg), aEncCfg, sizeof(UnionAudioEncCfg));
    librtmpk->bSendMeta = false;
    locker_unlock(&librtmpk->mutex);
    
    return;
}

/**
 @abstract 设置用户自定义的metadata
 
 @param publisher 推流对象
 @param char 关键字
 @param number 数值
 @param string 字符串
 */
void union_librtmpk_set_userMetadata(LiteLibrtmpk_t *librtmpk, char *key, double number, char *string)
{
    if(NULL == librtmpk || NULL == key)
        return ;
    
    UnionDictElem *pElem = NULL;
    librtmpk->userMetadata.elems  = (UnionDictElem *)realloc(librtmpk->userMetadata.elems, (librtmpk->userMetadata.number  + 1)* sizeof(UnionDictElem));
    if(librtmpk->userMetadata.elems)
    {
        pElem = &librtmpk->userMetadata.elems[librtmpk->userMetadata.number];
        memset(pElem, 0, sizeof(UnionDictElem));
        if(string)
        {
            pElem->val.string = (char *)(malloc(strlen(string) + 1));
            if(pElem->val.string)
            {
                memset(pElem->val.string, 0, strlen(string) + 1);
                strcpy(pElem->val.string, string);
            }
            pElem->type = UnionDataType_String;
        }
        else
        {
            pElem->type = UnionDataType_Number;
            pElem->val.number = number;
        }
        
        pElem->name = (uint8_t *)(malloc(strlen(key) + 1));
        if(pElem->name)
        {
            memset(pElem->name, 0, strlen(key) + 1);
            strcpy((char *)(pElem->name), key);
            librtmpk->userMetadata.number++;
        }
        else
        {
            if(pElem->val.string)
                free(pElem->val.string);
        }
    }
    
    return ;
}

static int union_librtmpk_parseurl(LiteLibrtmpk_t *librtmpk, const char *url)
{
	if (0 != rtmp_url_parse(url, &librtmpk->url))
		return UnionPublisher_Error_Invalid_Address;

	if (0 == strcmp("rtmp", librtmpk->url.scheme))
	{
		librtmpk->transport.open = tcp_open;
		librtmpk->transport.close = tcp_close;
		librtmpk->transport.write = tcp_write;
		librtmpk->transport.read = tcp_read;
	}
	else if (0 == strcmp("rtmpk", librtmpk->url.scheme))
	{
		librtmpk->transport.open = kcp_open;
		librtmpk->transport.close = kcp_close;
		librtmpk->transport.write = kcp_write;
		librtmpk->transport.read = kcp_read;
	}
	else
	{
		assert(0);
		return UnionPublisher_Error_Invalid_Address;
	}

	return 0;
}

/**
 * @abstract 开始推流
 */
int union_librtmpk_start(LiteLibrtmpk_t *librtmpk, const char *url, void *param, kcp_callback cb)
{
	uint8_t packet[2 * 1024];
    int errorCode = UnionPublisher_Error_Unknown;
    
	if (NULL == librtmpk)
		return errorCode;
    
	errorCode = union_librtmpk_parseurl(librtmpk, url);
	if (0 != errorCode)
		return errorCode;

	struct rtmp_client_handler_t handler;
	memset(&handler, 0, sizeof(handler));
	handler.send = rtmp_client_send;

    if(UnionPublisher_Status_Started != librtmpk->status)
    {
		librtmpk->rtmpHandle = rtmp_client_create(librtmpk->url.app, librtmpk->url.stream, librtmpk->url.tcurl, librtmpk, &handler);
		librtmpk->flvMuxer = flv_muxer_create(union_librtmpk_send_packet, librtmpk);
		if (!librtmpk->flvMuxer || !librtmpk->rtmpHandle)
		{
			errorCode = UnionPublisher_Error_Unknown;
			goto FAIL;
		}

		errorCode = librtmpk->transport.open(&librtmpk->kcp, librtmpk->url.host, librtmpk->url.port);
		if (0 != errorCode)
			goto FAIL;

		if (librtmpk->kcp.kcp)
		{
			librtmpk->kcp.kcp->notify = cb;
			librtmpk->kcp.kcp->notifyparam = param;
		}

		errorCode = rtmp_client_start(librtmpk->rtmpHandle, 0);
		while (0 == errorCode && 4 != rtmp_client_getstate(librtmpk->rtmpHandle) && (errorCode = librtmpk->transport.read(&librtmpk->kcp, packet, sizeof(packet), 0)) > 0)
		{
			errorCode = rtmp_client_input(librtmpk->rtmpHandle, packet, errorCode);
		}

		if (0 != errorCode)
		{
			if(librtmpk->kcp.kcp && librtmpk->kcp.kcp->netstat.clock)
				librtmpk->kcp.kcp->mtu = 500; // try to update rtt
			goto FAIL;
		}

		socket_getname(librtmpk->kcp.sock, librtmpk->local, &librtmpk->url.port);
		socket_addr_to((struct sockaddr*)&librtmpk->kcp.addr, librtmpk->kcp.addrlen, librtmpk->remote, &librtmpk->url.port);
		ip_route_get(librtmpk->remote, librtmpk->local);

        librtmpk->audioBaseTime = -1;
        librtmpk->videoBaseTime = -1;

        librtmpk->lastAudioTime = -1;
        librtmpk->lastVideoTime = -1;
        
        librtmpk->bInitAudio = false;
        librtmpk->bSendMeta = false;
        
        librtmpk->status = UnionPublisher_Status_Started;
    }
    
    return errorCode;

FAIL:
	union_librtmpk_stop(librtmpk);
    librtmpk->status = UnionPublisher_Status_Error;
    return errorCode;
}

/**
 * @abstract 发送数据包
 */
int union_librtmpk_send(LiteLibrtmpk_t *librtmpk, UnionAVPacket* packet)
{
    int ret = -1;
	uint8_t buffer[1024];
    
    if (NULL == librtmpk) {
        return ret;
    }
    if (NULL == packet || UnionPublisher_Status_Started != librtmpk->status) {
        goto FAIL;
    }

	// try to read peer data
	ret = librtmpk->transport.read(&librtmpk->kcp, buffer, sizeof(buffer), 1);
	if (ret > 0)
	{
		ret = rtmp_client_input(librtmpk->rtmpHandle, buffer, ret);
		if (ret < 0)
			return ret;
	}

    locker_lock(&librtmpk->mutex);
    
    if(!librtmpk->bSendMeta)
    {
        union_librtmpk_send_metadata(librtmpk);
        librtmpk->bSendMeta = true;
    }
    
    if(packet->type == UNION_MEDIA_TYPE_VIDEO)
        ret = union_librtmpk_send_videoframe(librtmpk, packet);
    else if(packet->type == UNION_MEDIA_TYPE_AUDIO)
        ret = union_librtmpk_send_audioframe(librtmpk, packet);
    locker_unlock(&librtmpk->mutex);
    
    if(ret >= 0)
        return ret;

FAIL:
	union_librtmpk_stop(librtmpk);
    librtmpk->status = UnionPublisher_Status_Error;
    return UnionPublisher_Error_Send_Failed;
}

void union_librtmpk_setTimeout(LiteLibrtmpk_t *librtmpk, int timeout) {
    if(NULL == librtmpk)
        return ;
	librtmpk->kcp.timeout = timeout;
}

/**
 * @abstract 停止推流
 */
void union_librtmpk_stop(LiteLibrtmpk_t *librtmpk)
{
    if(NULL == librtmpk)
        return ;

    locker_lock(&librtmpk->destroyMutex);
	if (librtmpk->kcp.sock != socket_invalid) {
		librtmpk->transport.close(&librtmpk->kcp);
		librtmpk->kcp.sock = socket_invalid;
	}
	if (librtmpk->rtmpHandle) {
		rtmp_client_destroy(librtmpk->rtmpHandle);
		librtmpk->rtmpHandle = NULL;
	}
	if (librtmpk->flvMuxer)
	{
		flv_muxer_destroy(librtmpk->flvMuxer);
		librtmpk->flvMuxer = NULL;
	}
    locker_unlock(&librtmpk->destroyMutex);
   
    librtmpk->status = UnionPublisher_Status_Stopped;
    return ;
}

/**
 * @abstract 销毁推流器 
 */
void union_librtmpk_close(LiteLibrtmpk_t *librtmpk)
{
    if(NULL == librtmpk)
        return ;
    
    union_librtmpk_stop(librtmpk);
    union_librtmpk_free_dict(&librtmpk->userMetadata);
    
    locker_destroy(&librtmpk->mutex);
	locker_destroy(&librtmpk->destroyMutex);
    
	if (librtmpk->audioBuffer)
	{
		free(librtmpk->audioBuffer);
		librtmpk->audioBuffer = NULL;
		librtmpk->audioCapacity = 0;
	}

    free(librtmpk);
    librtmpk = NULL;

    return ;
}

/**
 * @abstract 创建librtmpk推流器 
 */
LiteLibrtmpk_t *union_librtmpk_open()
{
    LiteLibrtmpk_t      *librtmpk = NULL;
    
    librtmpk = (LiteLibrtmpk_t *)(calloc(1, sizeof(LiteLibrtmpk_t)));
	if (NULL == librtmpk)
		return NULL;
    
	kcp_init(&librtmpk->kcp);

    union_librtmpk_set_default_videocfg(librtmpk);
    union_librtmpk_set_default_audiocfg(librtmpk);
    
	locker_create(&librtmpk->mutex);
	locker_create(&librtmpk->destroyMutex);
    
    librtmpk->status = UnionPublisher_Status_Idle;
    return librtmpk;
}

void union_librtmpk_set_kcp_parameter(LiteLibrtmpk_t *librtmpk, const KCPParameter_t* param)
{
	librtmpk->kcp.wnd_recv = param->wnd_recv;
	librtmpk->kcp.wnd_send = param->wnd_send;
	librtmpk->kcp.mtu_size = param->mtu_size;
	librtmpk->kcp.fast_ack = param->fast_ack;
	librtmpk->kcp.cong_lost = param->cong_lost;
	librtmpk->kcp.cong_band = param->cong_band;
	librtmpk->kcp.cong_incr = param->cong_incr;
	librtmpk->kcp.cong_decr = param->cong_decr;
}

void union_librtmpk_get_kcp_parameter(LiteLibrtmpk_t *librtmpk, KCPParameter_t* param)
{
	param->wnd_recv = librtmpk->kcp.wnd_recv;
	param->wnd_send = librtmpk->kcp.wnd_send;
	param->mtu_size = librtmpk->kcp.mtu_size;
	param->fast_ack = librtmpk->kcp.fast_ack;
	param->cong_lost = librtmpk->kcp.cong_lost;
	param->cong_band = librtmpk->kcp.cong_band;
	param->cong_incr = librtmpk->kcp.cong_incr;
	param->cong_decr = librtmpk->kcp.cong_decr;
}

/**
 * 获取本机IP
 */
char *union_librtmpk_get_local_ip_address(LiteLibrtmpk_t *librtmpk) {
    return librtmpk->local;
}

/**
 * 获取对端IP
 */
char *union_librtmpk_get_remote_ip_address(LiteLibrtmpk_t *librtmpk) {
    return librtmpk->remote;
}

unsigned int union_librtmpk_get_dns_time(LiteLibrtmpk_t *librtmpk)
{
	return librtmpk->kcp.dnsTime;
}

static int rtmp_client_send(void* param, const void* header, size_t len, const void* data, size_t bytes)
{
	int r = 0;
	LiteLibrtmpk_t *librtmpk = (LiteLibrtmpk_t*)param;
	if(0 == r && header && len > 0)
		r = librtmpk->transport.write(&librtmpk->kcp, (const uint8_t*)header, len);
	if (0 == r && data && bytes > 0)
		r = librtmpk->transport.write(&librtmpk->kcp, (const uint8_t*)data, bytes);
	return 0 == r ? len + bytes : r;
}
