/* Copyright (c) 2010-2011 Xiph.Org Foundation, Skype Limited
   Written by Jean-Marc Valin and Koen Vos */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdarg.h>
#include "celt.h"
#include "entenc.h"
#include "modes.h"
#include "API.h"
#include "stack_alloc.h"
#include "float_cast.h"
#include "opus.h"
#include "arch.h"
#include "pitch.h"
#include "opus_private.h"
#include "os_support.h"
#include "cpu_support.h"
#include "analysis.h"
#include "mathops.h"
#include "tuning_parameters.h"
//#define DISABLE_FLOAT_API 0
#ifdef FIXED_POINT
#include "fixed/structs_FIX.h"
#else
#include "float/structs_FLP.h"
#endif

#define MAX_ENCODER_BUFFER 480

#ifndef DISABLE_FLOAT_API
#define PSEUDO_SNR_THRESHOLD 316.23f    /* 10^(25/10) */
#endif


typedef struct {
   opus_val32 XX, XY, YY;
   opus_val16 smoothed_width;
   opus_val16 max_follower;
} StereoWidthState;

struct OpusEncoder {
    int          celt_enc_offset;//celt״̬��ƫ�Ƶ�ַ
    int          silk_enc_offset;//silk״̬��ƫ�Ƶ�ַ
    silk_EncControlStruct silk_mode;//silk���Ʊ�����
    int          application;//opus��Ӧ��ģʽ
    int          channels;//������
    int          delay_compensation;//�ӳٲ���
    int          force_channels;//�û��̶�����������������Ϊ��������Ӧ������Ϊ��OPUS_AUTO����������ƵΪ������
    int          signal_type;//�����ź����ͣ�����ݴ˲���ȷ����Ƶ���������ʣ�voice_est��������ѡ��OPUS_AUTO��48�� OPUS_SIGNAL_VOICE��127�� OPUS_SIGNAL_MUSIC��0��
                                                                             //��������voice_est�ٴμ��㵱ǰ����֡�Ĵ���
    int          user_bandwidth;//�û��̶��Ĵ����������OPUS_AUTO��������ǿ�Ʊ�Ϊ��ֵ��OPUS_BANDWIDTH_NARROWBAND OPUS_BANDWIDTH_MEDIUMBAND OPUS_BANDWIDTH_WIDEBAND
    int          max_bandwidth;//������
    int          user_forced_mode;//�û�ָ����ģʽ
    int          voice_ratio;//silk��Ĭ��Ϊ-1����signal_type�ź�����δָ��ʱ������ָ����������
    opus_int32   Fs;//��Ƶ������
    int          use_vbr;//ʹ��vbrģʽ�����CBRģʽ����
    int          vbr_constraint; //����vbr�����(ʵʱʹ�ø���ȫ)
    int          variable_duration;//һ֡�����ʱ�䣬Ӧ�ú�֡��С���Ӧ����������֡��С����40ms��variable_duration����Ϊ20ms����Ὣ40ms�����ݵ���20ms�������൱����Ϣ�ж�ʧ����Щ��Ϣԭ������40msʱ���ڳ��أ����ڷŵ�20ms�ڣ��൱�ڼӿ�һ���Ĳ����ٶ�
    opus_int32   bitrate_bps;//ʵ�ʱ����ʣ��������������ģʽ�����Ӷȣ�������������20ms�ĵ�Ч�����ʣ���ͨ���˲�������Ŀ�������ֽ������˲���ʵ������Ĭ��ֵ�޹أ������user_bitrate_bps�ٴμ���
    opus_int32   user_bitrate_bps;//�����趨ΪOPUS_AUTO�������֡��Ҳ���Ǻ�֡��С��ص�������ʵ�ʱ�����Ҳ����bitrate_bps = 60*st->Fs/frame_size + st->Fs*st->channels��
                                         // OPUS_BITRATE_MAX��ʵ�ʱ����� = max_data_bytes*8*st->Fs/frame_size������ָ��������ֵ������ָ����ֵ�����˲��������ת����bitrate_bpsֵ
    int          lsb_depth;//����λ����silk��Ϊ16
    int          encoder_buffer;//���뵱ǰ֡ʱ������һ֡�����ݸ���
    int          lfe;//δ֪??
    int          arch;//��ǰ�ܹ�
    int          use_dtx;//ʹ�ò��������䣬����400ms�ľ����λᵥ����һ֡��ʾ����                 /* SILK �� CELT ��ͨ�� DTX  general DTX for both SILK and CELT */
#ifndef DISABLE_FLOAT_API
    TonalityAnalysisState analysis;
#endif

#define OPUS_ENCODER_RESET_START stream_channels
    int          stream_channels;//�����������
    opus_int16   hybrid_stereo_width_Q14;//����������Ч���Ĳ���
    opus_int32   variable_HP_smth2_Q15;//��ͨ�˲�����ϵ���������ʼ���˲���ȥ��Ƶ���У�������˲����Ľ���Ƶ��
    opus_val16   prev_HB_gain;//celt��ʹ�ã���һ֡���Ӵ�����
    opus_val32   hp_mem[4];//������ͨ�˲���״̬
    int          mode;//��ǰ�ı���ģʽ
    int          prev_mode;//��¼��һ֡��mode
    int          prev_channels;//��һ֡��������
    int          prev_framesize;//��һ֡��֡��С
    int          bandwidth;//��ǰ����
    /* Bandwidth determined automatically from the rate (before any other adjustment) */
    int          auto_bandwidth;//������Ƶ����Ĵ���ֻ������st->mode == MODE_CELT_ONLY || st->first || st->silk_mode.allowBandwidthSwitch�Ż��Զ�ѡ�����
    int          silk_bw_switch;//silk��ת������
    /* Sampling rate (at the API level) */
    int          first;//��ʼ��������״̬�ı�־
    opus_val16 * energy_masking;//�ڱ�ЧӦ����
    StereoWidthState width_mem;//����
    opus_val16   delay_buffer[MAX_ENCODER_BUFFER*2];//�ӳٻ��������
#ifndef DISABLE_FLOAT_API
    int          detected_bandwidth;
    int          nb_no_activity_frames;
    opus_val32   peak_signal_energy;
#endif
    int          nonfinal_frame; /* current frame is not the final in a packet */
    opus_uint32  rangeFinal;//CELT�������ὫһЩ�����У����Ϣ��ӵ����һ�����ݰ��У���ȷ���������ܹ���ȷ�ؽ���������������st->rangeFinal != 0ʱ��CELT���������������Щ������Ϣ���Ӷ����Լ�����������ʡ�
};

/* ��Ƶ�źŴ����е�״̬ת�Ʊ�10000, 1000, ( NB<->MB)  11000, 1000,  (MB<->WB )13500, 1000,(  WB<->SWB)14000, 2000, ( SWB<->FB )��һ�����м���ֵ���޼��䣩���ڶ������ͺ�ֵ�����м���ֵ֮��Ĳ��죩Transition tables for the voice and music. First column is themiddle (memoriless) threshold. The second column is the hysteresis���м���ֵ֮��Ĳ���(difference with the middle) */
static const opus_int32 mono_voice_bandwidth_thresholds[8] = {
        10000, 1000, /* NB<->MB */
        11000, 1000, /* MB<->WB */
        13500, 1000, /* WB<->SWB */
        14000, 2000, /* SWB<->FB */
};
static const opus_int32 mono_music_bandwidth_thresholds[8] = {
         9000, 1000, /* NB<->MB */
        10000, 1000, /* MB<->WB */
        11000, 1000, /* WB<->SWB */
        12000, 2000, /* SWB<->FB */
};
static const opus_int32 stereo_voice_bandwidth_thresholds[8] = {
        10000, 1000, /* NB<->MB */
        11000, 1000, /* MB<->WB */
        13500, 1000, /* WB<->SWB */
        14000, 2000, /* SWB<->FB */
};
static const opus_int32 stereo_music_bandwidth_thresholds[8] = {
         9000, 1000, /* NB<->MB */
        10000, 1000, /* MB<->WB */
        11000, 1000, /* WB<->SWB */
        12000, 2000, /* SWB<->FB */
};
/* Threshold bit-rates for switching between mono and stereo */
static const opus_int32 stereo_voice_threshold = 19000;
static const opus_int32 stereo_music_threshold = 17000;

/* Threshold bit-rate for switching between SILK/hybrid and CELT-only */
static const opus_int32 mode_thresholds[2][2] = {
      /* voice */ /* music */
      {  64000,      10000}, /* mono */
      {  44000,      10000}, /* stereo */
};

static const opus_int32 fec_thresholds[] = {
        12000, 1000, /* NB */
        14000, 1000, /* MB */
        16000, 1000, /* WB */
        20000, 1000, /* SWB */
        22000, 1000, /* FB */
};

int opus_encoder_get_size(int channels)
{
    int silkEncSizeBytes, celtEncSizeBytes;
    int ret;
    if (channels<1 || channels > 2)
        return 0;
    ret = silk_Get_Encoder_Size( &silkEncSizeBytes );//������sizeof�õ�silk_encoder�ṹ��Ĵ�Сλ17384�ֽ�
    if (ret)
        return 0;
    silkEncSizeBytes = align(silkEncSizeBytes);
    celtEncSizeBytes = celt_encoder_get_size(channels);//celtEncSizeBytes=4996
    return align(sizeof(OpusEncoder))+silkEncSizeBytes+celtEncSizeBytes;
}

int opus_encoder_init(OpusEncoder* st, opus_int32 Fs, int channels, int application)
{   
    //ָ�� SILK ������״̬��ָ��
    void *silk_enc;
    //ָ�� celt ������״̬��ָ��
    CELTEncoder *celt_enc;
    int err;
    int ret, silkEncSizeBytes;

   if((Fs!=48000&&Fs!=24000&&Fs!=16000&&Fs!=12000&&Fs!=8000)||(channels!=1&&channels!=2)||
        (application != OPUS_APPLICATION_VOIP && application != OPUS_APPLICATION_AUDIO
        && application != OPUS_APPLICATION_RESTRICTED_LOWDELAY))
        return OPUS_BAD_ARG;

    OPUS_CLEAR((char*)st, opus_encoder_get_size(channels));//�Ƚ�encoderָ��Ľṹ���ʼ������stָ����ָλ�ú����24588�ֽ�ȫ����ʼ��opus_encoder_get_size(channels)=24588�ֽ�
    /* Create SILK encoder */
    ret = silk_Get_Encoder_Size( &silkEncSizeBytes );//SILK�������ṹ���ʵ�ʴ�СsilkEncSizeBytes=17384����ʣ7204���ֽڣ��޲���򷵻�ret=0
    if (ret)
        return OPUS_BAD_ARG;
    silkEncSizeBytes = align(silkEncSizeBytes);//silkEncSizeBytes=17384
    st->silk_enc_offset = align(sizeof(OpusEncoder));//sizeof(OpusEncoder)=2208,st->silk_enc_offset=2208
    st->celt_enc_offset = st->silk_enc_offset+silkEncSizeBytes;//17384+2208=19592

    //encoder=st�ṹ����
    /*encoderһ��24588���ֽ�********************��2208���ֽ�******************************************************************��19592���ֽ�*********************************/
    /*      opus������״̬ռ��2208���ֽ�         silk_enc                �м�17384���ֽ���silk������״̬                      celt_enc   ����4996���ֽ���celt������״̬  */
    /***********************************************************************************************************************************************************************/

    silk_enc = (char*)st+st->silk_enc_offset;
    celt_enc = (CELTEncoder*)((char*)st+st->celt_enc_offset);
    st->stream_channels = st->channels = channels;//����������=opus������=��Ƶ������

    st->Fs = Fs;    //opus������������=��Ƶ������
    st->arch = opus_select_arch();//����һ������ֵ���Ǽܹ����ͣ�����x86��ARM��MIPS��

    ret = silk_InitEncoder( silk_enc, st->arch, &st->silk_mode );//����SILK ���������ɹ�����0
    if(ret)return OPUS_INTERNAL_ERROR;

    /* Ԥ��slik����������  default SILK parameters */
    st->silk_mode.nChannelsAPI              = channels;//����opus������������Ƶ������һ��
    st->silk_mode.nChannelsInternal         = channels;//opus�ڲ��������ݵ�����������Ƶ������һ��
    st->silk_mode.API_sampleRate            = st->Fs;//����opus����Ƶ������
    st->silk_mode.maxInternalSampleRate     = 16000;//���opus�ڲ������ʣ�silkonlyģʽ�»����¸�ֵΪ16000
    st->silk_mode.minInternalSampleRate     = 8000;//ԭ8000����Сopus�ڲ������ʣ�silkonlyģʽ�»����¸�ֵΪ8000
    st->silk_mode.desiredInternalSampleRate = 8000;//ԭ16000��Opus��������ڲ�ʹ�õĲ����ʣ�ֻ��8k��16k��12k����ѡ�񣬺�����ݵ�ǰ�������¸�ֵ�������խ����ѡ��8k������Ҫ���²���
    st->silk_mode.payloadSize_ms            = 20;//silk��������л��ٴμ��㣬ʹ����Ԥ���framesize��ƥ��       silk_onlyģʽ�� =1000*framesize/fs
    st->silk_mode.bitRate                   = 6000;//ԭ25k��ѹ������Ŀ������bps������������ֵ����ÿһ֡ѹ���������λ��
    st->silk_mode.packetLossPercentage      = 0;//ԭ0
    st->silk_mode.complexity                = 3;//1-10��ָ��opus�ڲ�����ĸ��Ӷȣ��������Ƶ�������ĸ��Ӷȣ���Ƶ����������ֵ����Ƶ�����а׻��˲����Ľ��������������˲����Ľ���
                                                               //��Ƶ����ѧϰ���������������о���״̬�������Ƿ����NLSF��NLSF_MSVQʣ��ĸ�����Ť���������ε�Ť��������ֵ
    st->silk_mode.useInBandFEC              = 0;//������FEC��ǰ�������Ҫ������������������֤���Ƿ����
    st->silk_mode.useDTX                    = 0;//0�򲻼�⾲���Σ�1���⾲���Σ�����400ms�򵥶���һ֡��־�˾�����
    st->silk_mode.useCBR                    = 0;//ԭ��0��ʹ�ú㶨������ģʽ
    st->silk_mode.reducedDependency         = 0;//֡��֮֡����أ�����Ϊ1���໥������������ÿһ֡Я������Ϣ��������

    /* ���첢��ʼ��celt������Create CELT encoder */
    /* Initialize CELT encoder */
    err = celt_encoder_init(celt_enc, Fs, channels, st->arch);//��ʼ��celt��������Ԥ��celt��������һ���ֲ���������ֻ����������ѹ����˲���ϸע��celt

    if(err!=OPUS_OK)return OPUS_INTERNAL_ERROR;

    celt_encoder_ctl(celt_enc, CELT_SET_SIGNALLING(0));//���ڹر����signalling��ѡ�����ѡ��������Ƶ������������������һЩ�������Ϣ���԰������������õؽ�����Ƶ���ݡ���Щ��Ϣ���԰���ͨ�����á�λ��ȡ������ʵȡ�
                                                       //���磺����������ͽ�����֮����ŵ��ǳ��ȶ������Ҳ�����ͬ�����ã�����Ҫ����������Ϣ����Ϊ�������Ѿ�֪����Щ��Ϣ��
    celt_encoder_ctl(celt_enc, OPUS_SET_COMPLEXITY(st->silk_mode.complexity));//��st->silk_mode.complexityǿ��ת��Ϊopus_int32���ͣ����x����0���򲻻ᷢ���κ�����

    /* Ԥ��opus����������*/
    st->use_vbr = 1;//ʹ��vbrģʽ�����CBRģʽ����
    st->vbr_constraint = 1;//ԭ1           /* ����vbr�����(ʵʱʹ�ø���ȫ)Makes constrained VBR the default (safer for real-time use) */
    st->user_bitrate_bps = 6000;//�����趨ΪOPUS_AUTO�������֡��Ҳ���Ǻ�֡��С��ص�������ʵ�ʱ�����Ҳ����bitrate_bps = 60*st->Fs/frame_size + st->Fs*st->channels��
                                         // OPUS_BITRATE_MAX��ʵ�ʱ����� = max_data_bytes*8*st->Fs/frame_size������ָ��������ֵ������ָ����ֵ�����˲��������ת����bitrate_bpsֵ
    st->bitrate_bps =  Fs * channels;//ʵ�ʱ����ʣ��������������ģʽ�����Ӷȣ�������������20ms�ĵ�Ч�����ʣ���ͨ���˲�������Ŀ�������ֽ������˲���ʵ������Ĭ��ֵ�޹أ������user_bitrate_bps�ٴμ���
    st->application = application;//��Ӧopus��Ӧ��ģʽ
    st->signal_type = OPUS_SIGNAL_VOICE;//�����ź����ͣ�����ݴ˲���ȷ����Ƶ������ˮƽ��voice_est��������ѡ��OPUS_AUTO��48�� OPUS_SIGNAL_VOICE��127�� OPUS_SIGNAL_MUSIC��0��
                                                                             //��������voice_est�ٴμ��㵱ǰ����֡�Ĵ���
    st->user_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//�û��̶��Ĵ����������OPUS_AUTO��������ǿ�Ʊ�Ϊ��ֵ��OPUS_BANDWIDTH_NARROWBAND OPUS_BANDWIDTH_MEDIUMBAND OPUS_BANDWIDTH_WIDEBAND
    st->max_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//�������ޣ�ʵ�ʴ����С�ڴ˲���
    st->force_channels = OPUS_AUTO;//�û��̶�����������������Ϊ��������Ӧ������Ϊ��OPUS_AUTO����������ƵΪ������
    st->user_forced_mode = MODE_SILK_ONLY;//���û��ǿ���趨ģʽOPUS_AUTO��������signal_type�����voice_est������������ֵı����ʻ���ֵ��������������ޣ���ѡ��ģʽ��
                                          //���ǿ���趨��ģʽ�򲻻�������ϣ�ֱ��ǿ���趨Ϊ��ģʽ MODE_HYBRID  MODE_CELT_ONLY MODE_SILK_ONLY
    st->voice_ratio = 1;//�ڱ�������л�����Ϊ-1�����OPUS_APPLICATION_AUDIOģʽ������voice_est
    st->encoder_buffer = st->Fs / 100;//���뵱ǰ֡ʱ��������һ֡����Ƶ���ݵĸ���������������ӳ�
    st->lsb_depth =16;     //�������ȣ�Ҳ���ǲ���λ������Ҫ����celt��silk��Ĭ��16
    st->variable_duration = OPUS_FRAMESIZE_ARG;//Ĭ�����ò�������ѡ��֡��С�Ĳ�����ԭOPUS_FRAMESIZE_ARG
    st->delay_compensation = st->Fs/250;/* �ӳٲ���4����(SILK �Ķ���ǰհ2.5���� + SILK �ز���������Ԥ��1.5����)Delay compensation of 4 ms (2.5 ms for SILK's extra look-ahead+1.5 ms for SILK resamplers and stereo prediction) */
    st->hybrid_stereo_width_Q14 = 1 << 14;//����������Ч���Ĳ���
    st->prev_HB_gain = Q15ONE;//��һ֡���Ӵ����棬��Ҫ����celt�ж���
    st->variable_HP_smth2_Q15 = silk_LSHIFT( silk_lin2log( VARIABLE_HP_MIN_CUTOFF_HZ ), 8 );//��ͨ�˲�����ϵ���������ʼ���˲���ȥ��Ƶ���У�������˲����Ľ���Ƶ��
    st->first = 1;//��ʼ��������״̬�ı�־��ֻ�е�һ֡��1�����߷�������ָ��ʱ
    st->mode = MODE_SILK_ONLY;//��ǰ�ı���ģʽ��Ĭ������£�CELT�������ͽ�������ģʽ��ΪMODE_HYBRID������Ӧ�ó��������󣬿���ѡ��ͬ��ģʽ�Դﵽ��ѵ���Ƶ���������ݴ���Ч��
    st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;//��ǰ����

#ifndef DISABLE_FLOAT_API
    tonality_analysis_init(&st->analysis, st->Fs);
    st->analysis.application = st->application;
#endif

    return OPUS_OK;
}

static unsigned char gen_toc(int mode, int framerate, int bandwidth, int channels)
{
   int period;
   unsigned char toc;
   period = 0;
   while (framerate < 400)
   {
       framerate <<= 1;
       period++;
   }
   if (mode == MODE_SILK_ONLY)
   {
       toc = (bandwidth-OPUS_BANDWIDTH_NARROWBAND)<<5;
       toc |= (period-2)<<3;
   } else if (mode == MODE_CELT_ONLY)
   {
       int tmp = bandwidth-OPUS_BANDWIDTH_MEDIUMBAND;
       if (tmp < 0)
           tmp = 0;
       toc = 0x80;
       toc |= tmp << 5;
       toc |= period<<3;
   } else
       /* Hybrid */
   {
       toc = 0x60;
       toc |= (bandwidth-OPUS_BANDWIDTH_SUPERWIDEBAND)<<4;
       toc |= (period-2)<<3;
   }
   toc |= (channels==2)<<2;
   return toc;
}

#ifndef FIXED_POINT
static void silk_biquad_float(
    const opus_val16      *in,            /* I:    Input signal                   */
    const opus_int32      *B_Q28,         /* I:    MA coefficients [3]            */
    const opus_int32      *A_Q28,         /* I:    AR coefficients [2]            */
    opus_val32            *S,             /* I/O:  State vector [2]               */
    opus_val16            *out,           /* O:    Output signal                  */
    const opus_int32      len,            /* I:    Signal length (must be even)   */
    int stride
)
{
    /* DIRECT FORM II TRANSPOSED (uses 2 element state vector) */
    opus_int   k;
    opus_val32 vout;
    opus_val32 inval;
    opus_val32 A[2], B[3];

    A[0] = (opus_val32)(A_Q28[0] * (1.f/((opus_int32)1<<28)));
    A[1] = (opus_val32)(A_Q28[1] * (1.f/((opus_int32)1<<28)));
    B[0] = (opus_val32)(B_Q28[0] * (1.f/((opus_int32)1<<28)));
    B[1] = (opus_val32)(B_Q28[1] * (1.f/((opus_int32)1<<28)));
    B[2] = (opus_val32)(B_Q28[2] * (1.f/((opus_int32)1<<28)));

    /* Negate A_Q28 values and split in two parts */

    for( k = 0; k < len; k++ ) {
        /* S[ 0 ], S[ 1 ]: Q12 */
        inval = in[ k*stride ];
        vout = S[ 0 ] + B[0]*inval;

        S[ 0 ] = S[1] - vout*A[0] + B[1]*inval;

        S[ 1 ] = - vout*A[1] + B[2]*inval + VERY_SMALL;

        /* Scale back to Q0 and saturate */
        out[ k*stride ] = vout;
    }
}
#endif

static void hp_cutoff(const opus_val16 *in, opus_int32 cutoff_Hz, opus_val16 *out, opus_val32 *hp_mem, int len, int channels, opus_int32 Fs, int arch)
{
   opus_int32 B_Q28[ 3 ], A_Q28[ 2 ];
   opus_int32 Fc_Q19, r_Q28, r_Q22;//Fc_Q19=18525
   (void)arch;
   silk_assert( cutoff_Hz <= silk_int32_MAX / SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 ) );//���cutoff_Hz�Ƿ���ȷ SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 )=2471���cutoff_Hz�Ƿ�С��(2147483647/2470)=869426
   
   Fc_Q19 = silk_DIV32_16( silk_SMULBB( SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 ), cutoff_Hz ), Fs/1000 );//18532  =2471*cutoff_Hz/Fs/1000
   silk_assert( Fc_Q19 > 0 && Fc_Q19 < 32768 );//���Fc_Q19��0��32768֮��

   r_Q28 = SILK_FIX_CONST( 1.0, 28 ) - silk_MUL( SILK_FIX_CONST( 0.92, 9 ), Fc_Q19 );//259706884
   //�ֶ�ѡ��AR��MAϵ��
   /* b = r * [ 1; -2; 1 ]; */
   /* a = [ 1; -2 * r * ( 1 - 0.5 * Fc^2 ); r^2 ]; */
   B_Q28[ 0 ] = r_Q28;
   B_Q28[ 1 ] = silk_LSHIFT( -r_Q28, 1 );//-519413768
   B_Q28[ 2 ] = r_Q28;
   /* -r * ( 2 - Fc * Fc ); */
   r_Q22  = silk_RSHIFT( r_Q28, 6 );// r_Q22=4055646
   A_Q28[0] = silk_SMULWW(r_Q22, silk_SMULWW(Fc_Q19, Fc_Q19) - SILK_FIX_CONST(2.0, 22));//((r_Q22* (Fc_Q19* Fc_Q19 >> 16) - (2.0 * 1 << 22 + 0.5) >> 16)       A_Q28[ 0 ]=-518787523
   A_Q28[ 1 ] = silk_SMULWW( r_Q22, r_Q22 );//r_Q22* r_Q22 >> 16     A_Q28[ 0 ]=-519089305��A_Q28[ 1 ]=251262126
#ifdef FIXED_POINT
   if( channels == 1 ) {
      silk_biquad_alt_stride1( in, B_Q28, A_Q28, hp_mem, out, len );
   } else {
      silk_biquad_alt_stride2( in, B_Q28, A_Q28, hp_mem, out, len, arch );
   }
#else
   silk_biquad_float( in, B_Q28, A_Q28, hp_mem, out, len, channels );
   if( channels == 2 ) {
       silk_biquad_float( in+1, B_Q28, A_Q28, hp_mem+2, out+1, len, channels );
   }
#endif
}

#ifdef FIXED_POINT
static void dc_reject(const opus_val16 *in, opus_int32 cutoff_Hz, opus_val16 *out, opus_val32 *hp_mem, int len, int channels, opus_int32 Fs)
{
   int c, i;
   int shift;

   /* Approximates -round(log2(6.3*cutoff_Hz/Fs)) */
   shift=celt_ilog2(Fs/(cutoff_Hz*4));
   for (c=0;c<channels;c++)
   {
      for (i=0;i<len;i++)
      {
         opus_val32 x, y;
         x = SHL32(EXTEND32(in[channels*i+c]), 14);
         y = x-hp_mem[2*c];
         hp_mem[2*c] = hp_mem[2*c] + PSHR32(x - hp_mem[2*c], shift);
         out[channels*i+c] = EXTRACT16(SATURATE(PSHR32(y, 14), 32767));
      }
   }
}

#else
static void dc_reject(const opus_val16 *in, opus_int32 cutoff_Hz, opus_val16 *out, opus_val32 *hp_mem, int len, int channels, opus_int32 Fs)
{
   int i;
   float coef, coef2;
   coef = 6.3f*cutoff_Hz/Fs;
   coef2 = 1-coef;
   if (channels==2)
   {
      float m0, m2;
      m0 = hp_mem[0];
      m2 = hp_mem[2];
      for (i=0;i<len;i++)
      {
         opus_val32 x0, x1, out0, out1;
         x0 = in[2*i+0];
         x1 = in[2*i+1];
         out0 = x0-m0;
         out1 = x1-m2;
         m0 = coef*x0 + VERY_SMALL + coef2*m0;
         m2 = coef*x1 + VERY_SMALL + coef2*m2;
         out[2*i+0] = out0;
         out[2*i+1] = out1;
      }
      hp_mem[0] = m0;
      hp_mem[2] = m2;
   } else {
      float m0;
      m0 = hp_mem[0];
      for (i=0;i<len;i++)
      {
         opus_val32 x, y;
         x = in[i];
         y = x-m0;
         m0 = coef*x + VERY_SMALL + coef2*m0;
         out[i] = y;
      }
      hp_mem[0] = m0;
   }
}
#endif

static void stereo_fade(const opus_val16 *in, opus_val16 *out, opus_val16 g1, opus_val16 g2,
        int overlap48, int frame_size, int channels, const opus_val16 *window, opus_int32 Fs)
{
    int i;
    int overlap;
    int inc;
    inc = 48000/Fs;
    overlap=overlap48/inc;
    g1 = Q15ONE-g1;
    g2 = Q15ONE-g2;
    for (i=0;i<overlap;i++)
    {
       opus_val32 diff;
       opus_val16 g, w;
       w = MULT16_16_Q15(window[i*inc], window[i*inc]);
       g = SHR32(MAC16_16(MULT16_16(w,g2),
             Q15ONE-w, g1), 15);
       diff = EXTRACT16(HALF32((opus_val32)in[i*channels] - (opus_val32)in[i*channels+1]));
       diff = MULT16_16_Q15(g, diff);
       out[i*channels] = out[i*channels] - diff;
       out[i*channels+1] = out[i*channels+1] + diff;
    }
    for (;i<frame_size;i++)
    {
       opus_val32 diff;
       diff = EXTRACT16(HALF32((opus_val32)in[i*channels] - (opus_val32)in[i*channels+1]));
       diff = MULT16_16_Q15(g2, diff);
       out[i*channels] = out[i*channels] - diff;
       out[i*channels+1] = out[i*channels+1] + diff;
    }
}

static void gain_fade(const opus_val16 *in, opus_val16 *out, opus_val16 g1, opus_val16 g2,
        int overlap48, int frame_size, int channels, const opus_val16 *window, opus_int32 Fs)
{
    int i;
    int inc;
    int overlap;
    int c;
    inc = 48000/Fs;
    overlap=overlap48/inc;
    if (channels==1)
    {
       for (i=0;i<overlap;i++)
       {
          opus_val16 g, w;
          w = MULT16_16_Q15(window[i*inc], window[i*inc]);
          g = SHR32(MAC16_16(MULT16_16(w,g2),
                Q15ONE-w, g1), 15);
          out[i] = MULT16_16_Q15(g, in[i]);
       }
    } else {
       for (i=0;i<overlap;i++)
       {
          opus_val16 g, w;
          w = MULT16_16_Q15(window[i*inc], window[i*inc]);
          g = SHR32(MAC16_16(MULT16_16(w,g2),
                Q15ONE-w, g1), 15);
          out[i*2] = MULT16_16_Q15(g, in[i*2]);
          out[i*2+1] = MULT16_16_Q15(g, in[i*2+1]);
       }
    }
    c=0;
    do {
       for (i=overlap;i<frame_size;i++)
       {
          out[i*channels+c] = MULT16_16_Q15(g2, in[i*channels+c]);
       }
    }
    while (++c<channels);
}

//�����������ʼ�������ط����ڴ���׵�ַ����̬�����ַ������24588��byte�������׵�ַ����
OpusEncoder *opus_encoder_create(opus_int32 Fs, int channels, int application, int *error)
{
   int ret;
   OpusEncoder *st;//OpusEncoder���͵�ָ�����
   if((Fs!=48000&&Fs!=24000&&Fs!=16000&&Fs!=12000&&Fs!=8000)||(channels!=1&&channels!=2)||
       (application != OPUS_APPLICATION_VOIP && application != OPUS_APPLICATION_AUDIO
       && application != OPUS_APPLICATION_RESTRICTED_LOWDELAY))//�����ʲ���48000��24000��16000��12000��8000�����ߣ��������ϲ�Ϊ1��2�����߱�����ģʽ����������ģʽ���ؿ�ָ��Ҳ�㱨��
   {  
      if (error)
         *error = OPUS_BAD_ARG;
      return NULL;
   }
   st = (OpusEncoder *)opus_alloc(opus_encoder_get_size(channels));//��̬�����ַ������24588��byte�������׵�ַ��ֵ��stָ��
   if (st == NULL)
   {
      if (error)
      {
          *error = OPUS_ALLOC_FAIL;
          printf("��̬����ʧ��");//
      }

      return NULL;
   }
   ret = opus_encoder_init(st, Fs, channels, application);//����ͬʱ��ʼ��celt��silk�ı�����״̬,�Լ�Ԥ��������Ĳ���
                                                         //��ʼ��������״̬�������Ƿ�ɹ�OPUS_OK��errors
   //printf("*st->variable_duration= %d ", st->variable_duration);�����5000
   if (error)
      *error = ret;
   if (ret != OPUS_OK)
   {
      opus_free(st);//����ʧ���ͷ��ڴ�
      st = NULL;
   }
   return st;
}
//���û��趨������ת��Ϊʵ�ʱ�����OPUS_AUTO��60*st->Fs/frame_size + st->Fs*st->channels����OPUS_BITRATE_MAX��max_data_bytes*8*st->Fs/frame_size����������ֱ�Ӹ�ֵ
static opus_int32 user_bitrate_to_bitrate(OpusEncoder *st, int frame_size, int max_data_bytes)
{
  if(!frame_size)frame_size=st->Fs/400;//Ĭ��5msһ֡
  if (st->user_bitrate_bps==OPUS_AUTO)
    return 60*st->Fs/frame_size + st->Fs*st->channels;
  else if (st->user_bitrate_bps==OPUS_BITRATE_MAX)
    return max_data_bytes*8*st->Fs/frame_size;
  else
    return st->user_bitrate_bps;
}

#ifndef DISABLE_FLOAT_API
#ifdef FIXED_POINT
#define PCM2VAL(x) FLOAT2INT16(x)
#else
#define PCM2VAL(x) SCALEIN(x)
#endif

void downmix_float(const void *_x, opus_val32 *y, int subframe, int offset, int c1, int c2, int C)
{
   const float *x;
   int j;

   x = (const float *)_x;
   for (j=0;j<subframe;j++)
      y[j] = PCM2VAL(x[(j+offset)*C+c1]);
   if (c2>-1)
   {
      for (j=0;j<subframe;j++)
         y[j] += PCM2VAL(x[(j+offset)*C+c2]);
   } else if (c2==-2)
   {
      int c;
      for (c=1;c<C;c++)
      {
         for (j=0;j<subframe;j++)
            y[j] += PCM2VAL(x[(j+offset)*C+c]);
      }
   }
}
#endif

void downmix_int(const void *_x, opus_val32 *y, int subframe, int offset, int c1, int c2, int C)
{
   const opus_int16 *x;
   int j;

   x = (const opus_int16 *)_x;
   for (j=0;j<subframe;j++)
      y[j] = x[(j+offset)*C+c1];
   if (c2>-1)
   {
      for (j=0;j<subframe;j++)
         y[j] += x[(j+offset)*C+c2];
   } else if (c2==-2)
   {
      int c;
      for (c=1;c<C;c++)
      {
         for (j=0;j<subframe;j++)
            y[j] += x[(j+offset)*C+c];
      }
   }
}

opus_int32 frame_size_select(opus_int32 frame_size, int variable_duration, opus_int32 Fs)
{
    //variable_duration=5000    Ĭ�ϲ���,�������ֻ��������֣�Ŀ�ľ���ѡ������֡��������Ǻ��趨��֡����ƥ��Ҳ������ȣ��������ֱ��ٲ����������½�������ԭ����OpusEncoder�ṹ����
       //variable_duration���ܴ������õ�֡��С����Ϊ
       int new_size;
       if (frame_size<Fs/400)  //֡��С��2.5msС
       {
           printf("frame_size<Fs/400");
           return -1;
       }
       if (variable_duration == OPUS_FRAMESIZE_ARG)   //�������variable_duration��Ĭ��ֵ��ѡ��֡����ֱ�Ӳ����趨��frame_size֡����������ݸ��ĵ�Ĭ��ֵѡ��֡��
           new_size = frame_size;
       else if (variable_duration >= OPUS_FRAMESIZE_2_5_MS && variable_duration <= OPUS_FRAMESIZE_120_MS)//variable_durationҲ�ǿ�����2.5ms��120ms֮��
       {                                            
           if (variable_duration <= OPUS_FRAMESIZE_40_MS)                          //����ļ�����Ϊ��ʵ�����¼����֡��С��variable_duration���õ�ֵ���
               new_size = (Fs / 400) << (variable_duration - OPUS_FRAMESIZE_2_5_MS);
           else
               new_size = (variable_duration - OPUS_FRAMESIZE_2_5_MS - 2) * Fs / 50;
       }
       else
           return -1;
       if (new_size>frame_size)                                                    //����variable_duration�������֡�����ᳬ���趨��֡��
          return -1;
       if (400*new_size!=Fs   && 200*new_size!=Fs   && 100*new_size!=Fs   &&
            50*new_size!=Fs   &&  25*new_size!=Fs   &&  50*new_size!=3*Fs &&
            50*new_size!=4*Fs &&  50*new_size!=5*Fs &&  50*new_size!=6*Fs)         //������֡���Ƿ����
          return -1;
       return new_size;
}

opus_val16 compute_stereo_width(const opus_val16 *pcm, int frame_size, opus_int32 Fs, StereoWidthState *mem)
{
   opus_val32 xx, xy, yy;
   opus_val16 sqrt_xx, sqrt_yy;
   opus_val16 qrrt_xx, qrrt_yy;
   int frame_rate;
   int i;
   opus_val16 short_alpha;

   frame_rate = Fs/frame_size;
   short_alpha = Q15ONE - MULT16_16(25, Q15ONE)/IMAX(50,frame_rate);
   xx=xy=yy=0;
   /* Unroll by 4. The frame size is always a multiple of 4 *except* for
      2.5 ms frames at 12 kHz. Since this setting is very rare (and very
      stupid), we just discard the last two samples. */
   for (i=0;i<frame_size-3;i+=4)
   {
      opus_val32 pxx=0;
      opus_val32 pxy=0;
      opus_val32 pyy=0;
      opus_val16 x, y;
      x = pcm[2*i];
      y = pcm[2*i+1];
      pxx = SHR32(MULT16_16(x,x),2);
      pxy = SHR32(MULT16_16(x,y),2);
      pyy = SHR32(MULT16_16(y,y),2);
      x = pcm[2*i+2];
      y = pcm[2*i+3];
      pxx += SHR32(MULT16_16(x,x),2);
      pxy += SHR32(MULT16_16(x,y),2);
      pyy += SHR32(MULT16_16(y,y),2);
      x = pcm[2*i+4];
      y = pcm[2*i+5];
      pxx += SHR32(MULT16_16(x,x),2);
      pxy += SHR32(MULT16_16(x,y),2);
      pyy += SHR32(MULT16_16(y,y),2);
      x = pcm[2*i+6];
      y = pcm[2*i+7];
      pxx += SHR32(MULT16_16(x,x),2);
      pxy += SHR32(MULT16_16(x,y),2);
      pyy += SHR32(MULT16_16(y,y),2);

      xx += SHR32(pxx, 10);
      xy += SHR32(pxy, 10);
      yy += SHR32(pyy, 10);
   }
#ifndef FIXED_POINT
   if (!(xx < 1e9f) || celt_isnan(xx) || !(yy < 1e9f) || celt_isnan(yy))
   {
      xy = xx = yy = 0;
   }
#endif
   mem->XX += MULT16_32_Q15(short_alpha, xx-mem->XX);
   mem->XY += MULT16_32_Q15(short_alpha, xy-mem->XY);
   mem->YY += MULT16_32_Q15(short_alpha, yy-mem->YY);
   mem->XX = MAX32(0, mem->XX);
   mem->XY = MAX32(0, mem->XY);
   mem->YY = MAX32(0, mem->YY);
   if (MAX32(mem->XX, mem->YY)>QCONST16(8e-4f, 18))
   {
      opus_val16 corr;
      opus_val16 ldiff;
      opus_val16 width;
      sqrt_xx = celt_sqrt(mem->XX);
      sqrt_yy = celt_sqrt(mem->YY);
      qrrt_xx = celt_sqrt(sqrt_xx);
      qrrt_yy = celt_sqrt(sqrt_yy);
      /* Inter-channel correlation */
      mem->XY = MIN32(mem->XY, sqrt_xx*sqrt_yy);
      corr = SHR32(frac_div32(mem->XY,EPSILON+MULT16_16(sqrt_xx,sqrt_yy)),16);
      /* Approximate loudness difference */
      ldiff = MULT16_16(Q15ONE, ABS16(qrrt_xx-qrrt_yy))/(EPSILON+qrrt_xx+qrrt_yy);
      width = MULT16_16_Q15(celt_sqrt(QCONST32(1.f,30)-MULT16_16(corr,corr)), ldiff);
      /* Smoothing over one second */
      mem->smoothed_width += (width-mem->smoothed_width)/frame_rate;
      /* Peak follower */
      mem->max_follower = MAX16(mem->max_follower-QCONST16(.02f,15)/frame_rate, mem->smoothed_width);
   }
   /*printf("%f %f %f %f %f ", corr/(float)Q15ONE, ldiff/(float)Q15ONE, width/(float)Q15ONE, mem->smoothed_width/(float)Q15ONE, mem->max_follower/(float)Q15ONE);*/
   return EXTRACT16(MIN32(Q15ONE, MULT16_16(20, mem->max_follower)));
}

static int decide_fec(int useInBandFEC, int PacketLoss_perc, int last_fec, int mode, int *bandwidth, opus_int32 rate)
{
   int orig_bandwidth;
   if (!useInBandFEC || PacketLoss_perc == 0 || mode == MODE_CELT_ONLY)
      return 0;
   orig_bandwidth = *bandwidth;
   for (;;)
   {
      opus_int32 hysteresis;
      opus_int32 LBRR_rate_thres_bps;
      /* �����ڵ�ǰ����������ʹ��FEC����ֵCompute threshold for using FEC at the current bandwidth setting */
      LBRR_rate_thres_bps = fec_thresholds[2*(*bandwidth - OPUS_BANDWIDTH_NARROWBAND)];
      hysteresis = fec_thresholds[2*(*bandwidth - OPUS_BANDWIDTH_NARROWBAND) + 1];
      if (last_fec == 1) LBRR_rate_thres_bps -= hysteresis;
      if (last_fec == 0) LBRR_rate_thres_bps += hysteresis;
      LBRR_rate_thres_bps = silk_SMULWB( silk_MUL( LBRR_rate_thres_bps,
            125 - silk_min( PacketLoss_perc, 25 ) ), SILK_FIX_CONST( 0.01, 16 ) );
      /* If loss <= 5%, we look at whether we have enough rate to enable FEC.
         If loss > 5%, we decrease the bandwidth until we can enable FEC. */
      if (rate > LBRR_rate_thres_bps)
      {
          return 1;
          printf("����LBRR��������1");
      }
      else if (PacketLoss_perc <= 5)
         return 0;
      else if (*bandwidth > OPUS_BANDWIDTH_NARROWBAND)
         (*bandwidth)--;
      else
         break;
   }
   /* Couldn't find any bandwidth to enable FEC, keep original bandwidth. */
   *bandwidth = orig_bandwidth;
   return 0;
}

static int compute_silk_rate_for_hybrid(int rate, int bandwidth, int frame20ms, int vbr, int fec, int channels) {
   int entry;
   int i;
   int N;
   int silk_rate;
   static int rate_table[][5] = {
  /*  |total| |-------- SILK------------|
              |-- No FEC -| |--- FEC ---|
               10ms   20ms   10ms   20ms */
      {    0,     0,     0,     0,     0},
      {12000, 10000, 10000, 11000, 11000},
      {16000, 13500, 13500, 15000, 15000},
      {20000, 16000, 16000, 18000, 18000},
      {24000, 18000, 18000, 21000, 21000},
      {32000, 22000, 22000, 28000, 28000},
      {64000, 38000, 38000, 50000, 50000}
   };
   /* Do the allocation per-channel. */
   rate /= channels;
   entry = 1 + frame20ms + 2*fec;
   N = sizeof(rate_table)/sizeof(rate_table[0]);
   for (i=1;i<N;i++)
   {
      if (rate_table[i][0] > rate) break;
   }
   if (i == N)
   {
      silk_rate = rate_table[i-1][entry];
      /* For now, just give 50% of the extra bits to SILK. */
      silk_rate += (rate-rate_table[i-1][0])/2;
   } else {
      opus_int32 lo, hi, x0, x1;
      lo = rate_table[i-1][entry];
      hi = rate_table[i][entry];
      x0 = rate_table[i-1][0];
      x1 = rate_table[i][0];
      silk_rate = (lo*(x1-rate) + hi*(rate-x0))/(x1-x0);
   }
   if (!vbr)
   {
      /* Tiny boost to SILK for CBR. We should probably tune this better. */
      silk_rate += 100;
   }
   if (bandwidth==OPUS_BANDWIDTH_SUPERWIDEBAND)
      silk_rate += 300;
   silk_rate *= channels;
   /* Small adjustment for stereo (calibrated for 32 kb/s, haven't tried other bitrates). */
   if (channels == 2 && rate >= 12000)
      silk_rate -= 1000;
   return silk_rate;
}

/* ���ض�Ӧ��20ms ֡�ĵ�Ч�����ʣ�ÿ�λ���bitrate���м��㣬�漰֡�ʣ�ͨ������vbr��ģʽ�����Ӷȣ�������Returns the equivalent bitrate corresponding to 20 ms frames,
   complexity 10 VBR operation. */
static opus_int32 compute_equiv_rate(opus_int32 bitrate, int channels,
      int frame_rate, int vbr, int mode, int complexity, int loss)
{
   opus_int32 equiv;
   equiv = bitrate;//9000
   /* ���ǽ�С֡�Ŀ�����Take into account overhead from smaller frames. */
   equiv -= (40*channels+20)*(frame_rate - 50);            //֡������20ms�����ӱ�����
   /* CBR is about a 8% penalty for both SILK and CELT. */
   if (!vbr)
      equiv -= equiv/12;                                   //����cbrģʽ�����8%��ʵ�ʱ�����
   /* һ����˵�������Ի����10% �Ĳ���(��0��10)��Complexity makes about 10% difference (from 0 to 10) in general. */
   equiv = equiv * (90+complexity)/100;                    //���ݸ��Ӷ��ٴμ���ʵ�ʱ����ʣ�ȡ(90+complexity)/100���ı�����
   if (mode == MODE_SILK_ONLY || mode == MODE_HYBRID)      //�漰��silkģʽ�����loss��������ʵ�ʱ����ʣ�������Ӷ�С��2����ȡ0.8����ʵ�ʱ�����
                                                           //celtģʽ�Ҹ��Ӷ�С��5ȡ0.9����ʵ�ʱ�����
                                                           //OPUS_AUTO�����loss����ʵ�ʱ�����
   {
      /* SILK complexity 0-1 uses the non-delayed-decision NSQ, which
         costs about 20%. */
      if (complexity<2)
         equiv = equiv*4/5;
      equiv -= equiv*loss/(6*loss + 10);
   } else if (mode == MODE_CELT_ONLY) {
      /* CELT complexity 0-4 doesn't have the pitch filter, which costs
         about 10%. */
      if (complexity<5)
         equiv = equiv*9/10;
   } else {
      /* Mode not known yet */
      /* Half the SILK loss*/
      equiv -= equiv*loss/(12*loss + 20);//lose=0
   }
   return equiv;
}

#ifndef DISABLE_FLOAT_API

static int is_digital_silence(const opus_val16* pcm, int frame_size, int channels, int lsb_depth)
{
   int silence = 0;
   opus_val32 sample_max = 0;
#ifdef MLP_TRAINING
   return 0;
#endif
   sample_max = celt_maxabs16(pcm, frame_size*channels);

#ifdef FIXED_POINT
   silence = (sample_max == 0);
   (void)lsb_depth;
#else
   silence = (sample_max <= (opus_val16) 1 / (1 << lsb_depth));
#endif

   return silence;
}

#ifdef FIXED_POINT
static opus_val32 compute_frame_energy(const opus_val16 *pcm, int frame_size, int channels, int arch)
{
   int i;
   opus_val32 sample_max;
   int max_shift;
   int shift;
   opus_val32 energy = 0;
   int len = frame_size*channels;
   (void)arch;
   /* Max amplitude in the signal */
   sample_max = celt_maxabs16(pcm, len);

   /* Compute the right shift required in the MAC to avoid an overflow */
   max_shift = celt_ilog2(len);
   shift = IMAX(0, (celt_ilog2(sample_max) << 1) + max_shift - 28);

   /* Compute the energy */
   for (i=0; i<len; i++)
      energy += SHR32(MULT16_16(pcm[i], pcm[i]), shift);

   /* Normalize energy by the frame size and left-shift back to the original position */
   energy /= len;
   energy = SHL32(energy, shift);

   return energy;
}
#else
static opus_val32 compute_frame_energy(const opus_val16 *pcm, int frame_size, int channels, int arch)
{
   int len = frame_size*channels;
   return celt_inner_prod(pcm, pcm, len, arch)/len;
}
#endif

/* Decides if DTX should be turned on (=1) or off (=0) */
static int decide_dtx_mode(float activity_probability,    /* probability that current frame contains speech/music */
                           int *nb_no_activity_frames,    /* number of consecutive frames with no activity */
                           opus_val32 peak_signal_energy, /* peak energy of desired signal detected so far */
                           const opus_val16 *pcm,         /* input pcm signal */
                           int frame_size,                /* frame size */
                           int channels,
                           int is_silence,                 /* only digital silence detected in this frame */
                           int arch
                          )
{
   opus_val32 noise_energy;

   if (!is_silence)
   {
      if (activity_probability < DTX_ACTIVITY_THRESHOLD)  /* is noise */
      {
         noise_energy = compute_frame_energy(pcm, frame_size, channels, arch);

         /* but is sufficiently quiet */
         is_silence = peak_signal_energy >= (PSEUDO_SNR_THRESHOLD * noise_energy);
      }
   }

   if (is_silence)
   {
      /* The number of consecutive DTX frames should be within the allowed bounds */
      (*nb_no_activity_frames)++;

      if (*nb_no_activity_frames > NB_SPEECH_FRAMES_BEFORE_DTX)
      {
         if (*nb_no_activity_frames <= (NB_SPEECH_FRAMES_BEFORE_DTX + MAX_CONSECUTIVE_DTX))
            /* Valid frame for DTX! */
            return 1;
         else
            (*nb_no_activity_frames) = NB_SPEECH_FRAMES_BEFORE_DTX;
      }
   } else
      (*nb_no_activity_frames) = 0;

   return 0;
}

#endif

static opus_int32 encode_multiframe_packet(OpusEncoder *st,
                                           const opus_val16 *pcm,
                                           int nb_frames,
                                           int frame_size,
                                           unsigned char *data,
                                           opus_int32 out_data_bytes,
                                           int to_celt,
                                           int lsb_depth,
                                           int float_api)
{
   int i;
   int ret = 0;
   VARDECL(unsigned char, tmp_data);
   int bak_mode, bak_bandwidth, bak_channels, bak_to_mono;
   VARDECL(OpusRepacketizer, rp);
   int max_header_bytes;
   opus_int32 bytes_per_frame;
   opus_int32 cbr_bytes;
   opus_int32 repacketize_len;
   int tmp_len;
   ALLOC_STACK;

   /* Worst cases:
    * 2 frames: Code 2 with different compressed sizes
    * >2 frames: Code 3 VBR */
   max_header_bytes = nb_frames == 2 ? 3 : (2+(nb_frames-1)*2);

   if (st->use_vbr || st->user_bitrate_bps==OPUS_BITRATE_MAX)
      repacketize_len = out_data_bytes;
   else {
      cbr_bytes = 3*st->bitrate_bps/(3*8*st->Fs/(frame_size*nb_frames));
      repacketize_len = IMIN(cbr_bytes, out_data_bytes);
   }
   bytes_per_frame = IMIN(1276, 1+(repacketize_len-max_header_bytes)/nb_frames);

   ALLOC(tmp_data, nb_frames*bytes_per_frame, unsigned char);
   ALLOC(rp, 1, OpusRepacketizer);
   opus_repacketizer_init(rp);

   bak_mode = st->user_forced_mode;
   bak_bandwidth = st->user_bandwidth;
   bak_channels = st->force_channels;

   st->user_forced_mode = st->mode;
   st->user_bandwidth = st->bandwidth;
   st->force_channels = st->stream_channels;

   bak_to_mono = st->silk_mode.toMono;
   if (bak_to_mono)
      st->force_channels = 1;
   else
      st->prev_channels = st->stream_channels;

   for (i=0;i<nb_frames;i++)
   {
      st->silk_mode.toMono = 0;
      st->nonfinal_frame = i<(nb_frames-1);

      /* When switching from SILK/Hybrid to CELT, only ask for a switch at the last frame */
      if (to_celt && i==nb_frames-1)
         st->user_forced_mode = MODE_CELT_ONLY;

      tmp_len = opus_encode_native(st, pcm+i*(st->channels*frame_size), frame_size,
         tmp_data+i*bytes_per_frame, bytes_per_frame, lsb_depth, NULL, 0, 0, 0, 0,
         NULL, float_api);

      if (tmp_len<0)
      {
         RESTORE_STACK;
         return OPUS_INTERNAL_ERROR;
      }

      ret = opus_repacketizer_cat(rp, tmp_data+i*bytes_per_frame, tmp_len);

      if (ret<0)
      {
         RESTORE_STACK;
         return OPUS_INTERNAL_ERROR;
      }
   }

   ret = opus_repacketizer_out_range_impl(rp, 0, nb_frames, data, repacketize_len, 0, !st->use_vbr);

   if (ret<0)
   {
      RESTORE_STACK;
      return OPUS_INTERNAL_ERROR;
   }

   /* Discard configs that were forced locally for the purpose of repacketization */
   st->user_forced_mode = bak_mode;
   st->user_bandwidth = bak_bandwidth;
   st->force_channels = bak_channels;
   st->silk_mode.toMono = bak_to_mono;

   RESTORE_STACK;
   return ret;
}

static int compute_redundancy_bytes(opus_int32 max_data_bytes, opus_int32 bitrate_bps, int frame_rate, int channels)
{
   int redundancy_bytes_cap;
   int redundancy_bytes;
   opus_int32 redundancy_rate;
   int base_bits;
   opus_int32 available_bits;
   base_bits = (40*channels+20);

   /* Equivalent rate for 5 ms frames. */
   redundancy_rate = bitrate_bps + base_bits*(200 - frame_rate);
   /* For VBR, further increase the bitrate if we can afford it. It's pretty short
      and we'll avoid artefacts. */
   redundancy_rate = 3*redundancy_rate/2;
   redundancy_bytes = redundancy_rate/1600;

   /* Compute the max rate we can use given CBR or VBR with cap. */
   available_bits = max_data_bytes*8 - 2*base_bits;
   redundancy_bytes_cap = (available_bits*240/(240+48000/frame_rate) + base_bits)/8;
   redundancy_bytes = IMIN(redundancy_bytes, redundancy_bytes_cap);
   /* It we can't get enough bits for redundancy to be worth it, rely on the decoder PLC. */
   if (redundancy_bytes > 4 + 8*channels)
      redundancy_bytes = IMIN(257, redundancy_bytes);
   else
      redundancy_bytes = 0;
   return redundancy_bytes;
}

opus_int32 opus_encode_native(OpusEncoder *st,/*opus����״̬��*/
                              const opus_val16 *pcm, /*��Ƶ���ݣ�һ֡*/
                              int frame_size,/*��variable_durationƥ����֡��С*/
                              unsigned char *data, /*�������*/
                              opus_int32 out_data_bytes, /*������ݸ���*/
                              int lsb_depth,/*�������ȣ��̶�16*/
                              const void *analysis_pcm, /*��Ƶ���ݣ�һ֡��pcmһ��*/
                              opus_int32 analysis_size,/*�û����õ�֡��С*/ 
                              int c1,/*0*/ int c2,/*-2*/
                              int analysis_channels, /*������*/
                              downmix_func downmix, /*һ����������*/
                              int float_api/*0*/)
{
    void *silk_enc;//silk������״̬��silk������
    CELTEncoder *celt_enc;//celt������״̬��celt������
    int i;
    int ret=0;
    opus_int32 nBytes;//�������Ԫ��Ŀ
    ec_enc enc;//���ڴ洢ѹ�����ݱ�������״̬��Ϣ
    int bytes_target;//���ڴ洢��������Ŀ������ֽ���
    int prefill=0;//���ڿ��Ʊ�������ʼ״̬��һ�����ͱ���, 0�����������ĳ�ʼ״̬Ϊ����״̬
    int start_band = 0;//���ڿ��Ʊ�������ʼ�����Ƶ������һ�����ͱ���
    int redundancy = 0;//Ϊ 0 ʱ����������������������
    int redundancy_bytes = 0; /* Number of bytes to use for redundancy frame ����֡���ֽ���*/
    int celt_to_silk = 0;//�Ƿ� CELT ģʽת��Ϊ SILK ģʽ���б���,Ϊ 0���򲻽���ת��
    VARDECL(opus_val16, pcm_buf);//������һ�� opus_val16 ���͵ı��� pcm_buf�����ڴ洢��Ƶ PCM ���ݵĻ�������
    int nb_compr_bytes;// �洢�ľ��Ǿ���ѹ����� Opus ���ݵ��ֽ�����
    int to_celt = 0;//�Ƿ������ź�ת��Ϊ CELT ģʽ,ת���Ļ������ڱ��ֽϸ���Ƶ������ͬʱʵ�ָ��ߵ�ѹ���ʡ�ԭ0
    opus_uint32 redundant_rng = 0;//ָʾ�Ƿ������������
    int cutoff_Hz, hp_freq_smth1;//cutoff_Hz �� hp_freq_smth1 �������ڿ��Ƹ�ͨ�˲����Ľ�ֹƵ�ʺ�ƽ���̶ȡ�Ĭ������£�cutoff_Hz ��ֵΪ 150 Hz������ζ�ű�������ȥ�������ź��е��� 150 Hz �Ĳ��֡�Ĭ������£�hp_freq_smth1 ��ֵΪ 1����ʾʹ����򵥵�ƽ��������
    int voice_est; /* Probability of voice in Q7 *///Q7��������
    opus_int32 equiv_rate;//�洢���Ǳ�������������ʵ�ƽ��ֵ��������������������������ѹ��Ч�ʡ�
    int delay_compensation;//�������Ʊ������ڽ��б���ʱԤ���ӳٵĴ�С���Ӷ�ʹ�ý������ܹ�׼ȷ�ؽ����ӳٲ�����32
    int frame_rate;//��������֡�ʣ�Ҳ����ÿ���Ӵ������Ƶ֡��=16
    opus_int32 max_rate; /* Max bitrate we're allowed to use��ʹ�õ��������� *///���ڿ��Ʊ���������������163328
    int curr_bandwidth;//��ʾ��ǰ�ı������= st->bandwidth=1101<4khzNB
    opus_val16 HB_gain;//���ڿ��Ƹ�Ƶ���档����������⵽�����ź��д��ڸ�Ƶ����ʱ��������ͨ������HB_gain���������źŵĸ�Ƶ���֡�
    opus_int32 max_data_bytes;//��ʾOpus�������������������ֽ������������ò����������Ʊ�������������ݴ�С����ȷ����������ݲ��ᳬ��ָ��������ֽ�����1276
    int total_buffer;//��ʾ�������ڲ������л������ܹ��������ɶ��ٲ�����32
    opus_val16 stereo_width;//���ڿ����������źŵĿ��,ֻ���������ź���Ч�����ڵ������ź���û���κ�Ӱ��=0
    const CELTMode *celt_mode;///��CELT����������celt_enc�е�ģʽ��Ϣ���浽����celt_mode��
#ifndef DISABLE_FLOAT_API
    AnalysisInfo analysis_info;
    int analysis_read_pos_bak=-1;
    int analysis_read_subframe_bak=-1;
    int is_silence = 0;
#endif
    VARDECL(opus_val16, tmp_prefill);
    ALLOC_STACK;
    max_data_bytes = IMIN(1276, out_data_bytes);                         //ȷ���������ֽ���
    st->rangeFinal = 0;
    if (frame_size <= 0 || max_data_bytes <= 0)                          //���֡��С�����ѹ���ֽ�
    {
       RESTORE_STACK;
       printf("//֡��СС��=�����������ݵı�����С��=��//1234//\n");
       return OPUS_BAD_ARG;
    }

    /* Cannot encode 100 ms in 1 byte */
    if (max_data_bytes==1 && st->Fs==(frame_size*10))                   //���ѹ���ֽ�����̫С��
    {
      RESTORE_STACK;
      printf("//����ֽ���Ϊ1�Ҳ�����=frame_size*10ʱ����opus��buffer̫С//1242//\n");
      return OPUS_BUFFER_TOO_SMALL;
    }
                                                                        //��silk��celt��������λ��ָ��λ��
    silk_enc = (char*)st+st->silk_enc_offset;                           
    celt_enc = (CELTEncoder*)((char*)st+st->celt_enc_offset);

    if (st->application == OPUS_APPLICATION_RESTRICTED_LOWDELAY)        //������õ��ӳ�ģʽ���ӳ�buffer����Ϊ0�������������ģʽbuffer����Ĭ��Ϊ�ӳٲ�����ʼֵst->Fs/250��Ҳ�����ӳ�4ms
    {
        printf("�ﵽ����ӳ�����Ҫ�����������Ż�");
        delay_compensation = 0;
    }
    else
       delay_compensation = st->delay_compensation;
    
    lsb_depth = IMIN(lsb_depth, st->lsb_depth);                         //���ú���ʱlsb_depth=16����˳����ڳ�ʼ��ʱ�� st->lsb_depth���õĸ�С���������λ������16

    celt_encoder_ctl(celt_enc, CELT_GET_MODE(&celt_mode));              //��CELT������ģʽ��Ϣ���浽����celt_mode��

#ifndef DISABLE_FLOAT_API
    analysis_info.valid = 0;
#ifdef FIXED_POINT
    if (st->silk_mode.complexity >= 10 && st->Fs>=16000)
#else
    if (st->silk_mode.complexity >= 7 && st->Fs>=16000)
#endif
    {
       if (is_digital_silence(pcm, frame_size, st->channels, lsb_depth))
       {
          is_silence = 1;
          printf("���뾲���ж�\n");
       } else {
          analysis_read_pos_bak = st->analysis.read_pos;
          analysis_read_subframe_bak = st->analysis.read_subframe;
          run_analysis(&st->analysis, celt_mode, analysis_pcm, analysis_size, frame_size,
                c1, c2, analysis_channels, st->Fs,
                lsb_depth, downmix, &analysis_info);
       }

       /* Track the peak signal energy */
       if (!is_silence && analysis_info.activity_probability > DTX_ACTIVITY_THRESHOLD)
          st->peak_signal_energy = MAX32(MULT16_32_Q15(QCONST16(0.999f, 15), st->peak_signal_energy),
                compute_frame_energy(pcm, frame_size, st->channels, st->arch));
    }
#else
    (void)analysis_pcm;
    (void)analysis_size;
    (void)c1;
    (void)c2;
    (void)analysis_channels;
    (void)downmix;
#endif

#ifndef DISABLE_FLOAT_API
    /* Reset voice_ratio if this frame is not silent or if analysis is disabled.
     * Otherwise, preserve voice_ratio from the last non-silent frame */
    if (!is_silence)
      st->voice_ratio = -1;

    st->detected_bandwidth = 0;
    if (analysis_info.valid)
    {
       int analysis_bandwidth;
       if (st->signal_type == OPUS_AUTO)
       {
          float prob;
          if (st->prev_mode == 0)
             prob = analysis_info.music_prob;
          else if (st->prev_mode == MODE_CELT_ONLY)
             prob = analysis_info.music_prob_max;
          else
             prob = analysis_info.music_prob_min;
          st->voice_ratio = (int)floor(.5+100*(1-prob));
       }

       analysis_bandwidth = analysis_info.bandwidth;
       if (analysis_bandwidth<=12)
          st->detected_bandwidth = OPUS_BANDWIDTH_NARROWBAND;
       else if (analysis_bandwidth<=14)
          st->detected_bandwidth = OPUS_BANDWIDTH_MEDIUMBAND;
       else if (analysis_bandwidth<=16)
          st->detected_bandwidth = OPUS_BANDWIDTH_WIDEBAND;
       else if (analysis_bandwidth<=18)
          st->detected_bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND;
       else
          st->detected_bandwidth = OPUS_BANDWIDTH_FULLBAND;
    }
#else
    st->voice_ratio = -1;
#endif
    if (st->channels==2 && st->force_channels!=1)
       stereo_width = compute_stereo_width(pcm, frame_size, st->Fs, &st->width_mem);            //�����������źŵĿ�ȣ���������Ϊ0
    else
       stereo_width = 0;
    total_buffer = delay_compensation;                                                          //�ӳ�buffer���� ==st->Fs/250
    st->bitrate_bps = user_bitrate_to_bitrate(st, frame_size, max_data_bytes);                  //�����û��趨�ı����ʼ���ʵ�ʱ����ʣ����� user_bitrateΪOPUS_AUTO����OPUS_BITRATE_MAX������st->bitrate_bps=user_bitrate
    frame_rate = st->Fs / frame_size;                                                           //֡�ʣ�ȡ�������õ�֡��С�͵�ǰ��Ƶ�Ĳ����ʣ�1s���м�֡
    if (!st->use_vbr)                                                                           //����CBR������ʵ�ʱ����ʣ������������ֽ�
    {
       int cbrBytes;
       /* Multiply by 12 to make sure the division is exact. */
       int frame_rate12 = 12*st->Fs/frame_size;
       /* We need to make sure that "int" values always fit in 16 bits. */
       cbrBytes = IMIN( (12*st->bitrate_bps/8 + frame_rate12/2)/frame_rate12, max_data_bytes);//����cbr���������ֽ�
       st->bitrate_bps = cbrBytes*(opus_int32)frame_rate12*8/12;//����cbr����µı�����
       /* Make sure we provide at least one byte to avoid failing. */
       max_data_bytes = IMAX(1, cbrBytes);
       printf("ʹ��cbrģʽ");
    }
    if (max_data_bytes<3 || st->bitrate_bps < 3*frame_rate*8                                   //����ֽ�С��3����һ֡������������ֽڣ�����֡��С��50��Ҳ����һ֡С��20ms��ʱȫ���������С��300�ֽ�Ҳ���Ǳ�����С��2400�ռ�̫С������ PLC��֡
       || (frame_rate<50 && (max_data_bytes*frame_rate<300 || st->bitrate_bps < 2400)))

    {
       /*����ռ�̫С���޷������õ����飬������ PLC��֡ If the space is too low to do something useful, emit 'PLC' frames.*/
       int tocmode = st->mode;//����ģʽ��TOC mode��MODE_HYBRID
       int bw = st->bandwidth == 0 ? OPUS_BANDWIDTH_NARROWBAND : st->bandwidth;
       int packet_code = 0;
       int num_multiframes = 0;
       if (tocmode==0)
           tocmode = MODE_SILK_ONLY;
       if (frame_rate>100)
           tocmode = MODE_CELT_ONLY;
       /* 40 ms -> 2 x 20 ms if in CELT_ONLY or   */
       if (frame_rate==25 && tocmode!=MODE_SILK_ONLY)
       {
          frame_rate = 50;
          packet_code = 1;
       }
       /* >= 60 ms frames */
       if (frame_rate<=16)
       {
          /* 1 x 60 ms, 2 x 40 ms, 2 x 60 ms */
          if (out_data_bytes==1 || (tocmode==MODE_SILK_ONLY && frame_rate!=10))
          {
             tocmode = MODE_SILK_ONLY;
             packet_code = frame_rate <= 12;
             frame_rate = frame_rate == 12 ? 25 : 16;
          }
          else
          {
             num_multiframes = 50/frame_rate;
             frame_rate = 50;
             packet_code = 3;
          }
       }

       if(tocmode==MODE_SILK_ONLY&&bw>OPUS_BANDWIDTH_WIDEBAND)
          bw=OPUS_BANDWIDTH_WIDEBAND;
       else if (tocmode==MODE_CELT_ONLY&&bw==OPUS_BANDWIDTH_MEDIUMBAND)
          bw=OPUS_BANDWIDTH_NARROWBAND;
       else if (tocmode==MODE_HYBRID&&bw<=OPUS_BANDWIDTH_SUPERWIDEBAND)
          bw=OPUS_BANDWIDTH_SUPERWIDEBAND;

       data[0] = gen_toc(tocmode, frame_rate, bw, st->stream_channels);                                     //PLC֡������ Opus ����е� TOC �ֶ�
       data[0] |= packet_code;                                                                              //PLC֡�е�ǰ�����ÿ����Ƶ֡�ĳ�����Ϣ,packet_code ��ֵ�� data[0] �ĵ�λ
       ret = packet_code <= 1 ? 1 : 2;

       max_data_bytes = IMAX(max_data_bytes, ret);

       if (packet_code==3)
          data[1] = num_multiframes;

       if (!st->use_vbr)
       {
          ret = opus_packet_pad(data, ret, max_data_bytes);
          if (ret == OPUS_OK)
             ret = max_data_bytes;
          else
             ret = OPUS_INTERNAL_ERROR;
       }
       RESTORE_STACK;
       return ret;
    }                                                                                                         //PLC֡����


    max_rate = frame_rate*max_data_bytes*8;                                                                  //����õ������������

    
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->channels, st->Fs/frame_size,st->use_vbr, 0, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);/* ����ģʽ�ŵ��������ٴμ���ʵ�ʱ�����Equivalent 20-ms rate for mode/channel/bandwidth decisions */
    if (st->signal_type == OPUS_SIGNAL_VOICE)                                                               //st->signal_type��Ĭ��ֵ�Ҳ���仯�����ģʽ����OPUS_APPLICATION_VOIPѹ���������û���
        voice_est = 127;//�ź��趨Ϊ�������Ӧ��������Ϊ1
    else if (st->signal_type == OPUS_SIGNAL_MUSIC)
       voice_est = 0;//�ź��趨Ϊ�������Ӧ��������Ϊ0
    else if (st->voice_ratio >= 0)
    {
       voice_est = st->voice_ratio*327>>8;
       /* ����һ����Ƶֻ��90%�Ŀ����������ź�For AUDIO, never be more than 90% confident of having speech */
       if (st->application == OPUS_APPLICATION_AUDIO)
          voice_est = IMIN(voice_est, 115);
    } else if (st->application == OPUS_APPLICATION_VOIP)
       voice_est = 115;                                                                                //
    else
       voice_est = 48;                                                                                 //
    if (st->force_channels!=OPUS_AUTO && st->channels == 2)
    {
        st->stream_channels = st->force_channels;
    } else {
#ifdef FUZZING
       /* Random mono/stereo decision */
       if (st->channels == 2 && (rand()&0x1F)==0)
          st->stream_channels = 3-st->stream_channels;
#else
       /*Rate-dependent mono-stereo decision */
       if (st->channels == 2)
       {
          opus_int32 stereo_threshold;
          stereo_threshold = stereo_music_threshold + ((voice_est*voice_est*(stereo_voice_threshold-stereo_music_threshold))>>14);
          if (st->stream_channels == 2)
             stereo_threshold -= 1000;
          else
             stereo_threshold += 1000;
          st->stream_channels = (equiv_rate > stereo_threshold) ? 2 : 1;
       } else {
          st->stream_channels = st->channels;                                                          //�����ڲ�����������
       }
#endif
    }
    /* ����������ѡ�����ƽ��������Update equivalent rate for channels decision. */
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->stream_channels, st->Fs/frame_size,st->use_vbr, 0, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);//����Ϊ������ֵ����
    /* ����Ӧ�ó�����ź�����ѡ��ģʽMode selection depending on application and signal type */
    if (st->application == OPUS_APPLICATION_RESTRICTED_LOWDELAY)                                       //���ӳ�Ӧ��ģʽ�������celtģʽ
    {                                                                                                 //���û��ǿ���趨ģʽ����������������ж�ѡ��ģʽ���������������silk���������������celtģʽ
       st->mode = MODE_CELT_ONLY;
    } 
    else if (st->user_forced_mode == OPUS_AUTO)
    {
#ifdef FUZZING
       /* Random mode switching */
       if ((rand()&0xF)==0)
       {
          if ((rand()&0x1)==0)
             st->mode = MODE_CELT_ONLY;
          else
             st->mode = MODE_SILK_ONLY;
       } else {
          if (st->prev_mode==MODE_CELT_ONLY)
             st->mode = MODE_CELT_ONLY;
          else
             st->mode = MODE_SILK_ONLY;
       }
#else
        opus_int32 mode_voice;//������Ҳ������������ֵ
        opus_int32 mode_music;//˫����Ҳ�������ָ���ֵ��ͨ�����ʵ�ֵ����������
       opus_int32 threshold;                                                                           //����������

       /* ���������ȵĲ�ֵInterpolate based on stereo width */
       mode_voice = (opus_int32)(MULT16_32_Q15(Q15ONE-stereo_width,mode_thresholds[0][0])+ MULT16_32_Q15(stereo_width,mode_thresholds[1][0]));//������
       mode_music = (opus_int32)(MULT16_32_Q15(Q15ONE-stereo_width,mode_thresholds[1][1])+ MULT16_32_Q15(stereo_width,mode_thresholds[1][1]));//˫����
       /* ��������/���ָ��ʵĲ�ֵInterpolate based on speech/music probability */
       threshold = mode_music + ((voice_est*voice_est*(mode_voice-mode_music))>>14);
       /* ����һЩ���õ����ԣ��� VoIP �� SILK ��ƫ��  Bias towards SILK for VoIP because of some useful features */
       if (st->application == OPUS_APPLICATION_VOIP)                                                    //����voipģʽ�����޼�8000
          threshold += 8000;
       //printf("%f %d\n", stereo_width/(float)Q15ONE, threshold);
       /* Hysteresis */
       if (st->prev_mode == MODE_CELT_ONLY)                                                             //����celt�����޼�4000������ռ�Ƚϸߣ����Լ�ȥһЩ���������ԣ�
           threshold -= 4000;
       else if (st->prev_mode>0)                                                                        //���ǵ�һ֡�����޼�4000
           threshold += 4000;

       st->mode = (equiv_rate >= threshold) ? MODE_CELT_ONLY: MODE_SILK_ONLY;                           //��Ч�����ʴ���������celt����silk�������ı�����һ���С��
       /* When FEC is enabled and there's enough packet loss, use SILK */
       if (st->silk_mode.useInBandFEC && st->silk_mode.packetLossPercentage > (128-voice_est)>>4)       //������ö����ٷֱȲ�����ǰ�����FEC����silk
          st->mode = MODE_SILK_ONLY; 
       /* When encoding voice and DTX is enabled but the generalized DTX cannot be used,
          because of complexity and sampling frequency settings, switch to SILK DTX and
          set the encoder to SILK mode */
#ifndef DISABLE_FLOAT_API
       st->silk_mode.useDTX = st->use_dtx && !(analysis_info.valid || is_silence);
#else
       st->silk_mode.useDTX = st->use_dtx;                                        //use_dtxĬ��ֵ���ݸ� st->silk_mode,һ�㲻���ã�ͬʱ����dtx����������ֵvoice_est���Լ�max_data_bytes�����жϱ�֡���õ�ģʽ
#endif
       if (st->silk_mode.useDTX && voice_est > 100)
          st->mode = MODE_SILK_ONLY;
#endif

       /* If max_data_bytes represents less than 6 kb/s, switch to CELT-only mode */                 //������߼���framesize=fs*t(�����t��һ֡��ʱ��)   max_bytes*8/t<6000(������Ǵ���С��6kb/s),��֡�ʴ���50ʱ�򲻵�ʽ�ұ߱��9000
       if (max_data_bytes < (frame_rate > 50 ? 9000 : 6000)*frame_size / (st->Fs * 8))
          st->mode = MODE_CELT_ONLY;
    } else {
       st->mode = st->user_forced_mode;                                                             //�û�ǿ���趨��ģʽ��Ĭ��auto
    }

    /* ����ѡ����ģʽ����ȷ����������Ҫ���֡��СOverride the chosen mode to make sure we meet the requested frame size */
    if (st->mode != MODE_CELT_ONLY && frame_size < st->Fs/100)                                       //֡��СС��10ms��celt
        st->mode = MODE_CELT_ONLY;
    if (st->lfe)
       st->mode = MODE_CELT_ONLY;

    if (st->prev_mode > 0 &&((st->mode != MODE_CELT_ONLY && st->prev_mode == MODE_CELT_ONLY) ||(st->mode == MODE_CELT_ONLY && st->prev_mode != MODE_CELT_ONLY)))
    {                                                                                                //��ǰ֡���ǵ�һ֡������һ֡��ǰ֡��ģʽ��celt�Ż���celtתΪsilk��������1�������ֽ�
        redundancy = 1;
        celt_to_silk = (st->mode != MODE_CELT_ONLY);
        if (!celt_to_silk)
        {
            /* Switch to SILK/hybrid if frame size is 10 ms or more*/
            if (frame_size >= st->Fs/100)
            {
                st->mode = st->prev_mode;
                to_celt = 1;
            } else {
                redundancy=0;
            }
        }
    }
    /* �������֡ʱ������ֻ��Ҫ�������һ֡�л��� CELT��������ش������������ģʽ��Ӧ���ж�������-> ������ת����When encoding multiframes, we can ask for a switch to CELT only in the last frame. This switch
    * is processed above as the requested mode shouldn't interrupt stereo->mono transition. */
    if (st->stream_channels == 1 && st->prev_channels ==2 && st->silk_mode.toMono==0                                          
          && st->mode != MODE_CELT_ONLY && st->prev_mode != MODE_CELT_ONLY)
    {
       /* �ӳ���֡��������-> ������ת�������� SILK ������һ��ƽ���Ļ���Delay stereo->mono transition by two frames so that SILK can do a smooth downmix */
       st->silk_mode.toMono = 1;
       st->stream_channels = 2;
    } else {
       st->silk_mode.toMono = 0;
    }

    /* ����ģʽ���߸��µ�Ч���ʣ�����loss=0�Ҹ��Ӷ�>2����û�仯����10929��������Ӷ�<2�򷵻�8743   Update equivalent rate with mode decision. */
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->stream_channels, st->Fs/frame_size,st->use_vbr, st->mode, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);//���ݸ��Ӷȣ�֡�ʸ����ڲ�������
    
    if (st->mode != MODE_CELT_ONLY && st->prev_mode == MODE_CELT_ONLY)
    {
        silk_EncControlStruct dummy;
        silk_InitEncoder( silk_enc, st->arch, &dummy);
        prefill=1;
    }

    /* �Զ�(�������ʵ�)����ѡ��Automatic (rate-dependent) bandwidth selection */
    if (st->mode == MODE_CELT_ONLY || st->first || st->silk_mode.allowBandwidthSwitch)              //�����������voice_est�Ľ����opus_auto)���¼���������������
    {
        const opus_int32 *voice_bandwidth_thresholds, *music_bandwidth_thresholds;//�����������ѡ�����޴��ݸ�����������voice_bandwidth_thresholds��music_bandwidth_thresholds
        opus_int32 bandwidth_thresholds[8];
        int bandwidth = OPUS_BANDWIDTH_FULLBAND;//Ĭ��1105<20 kHzȫƵ��
        if (st->channels==2 && st->force_channels!=1)
        {
           voice_bandwidth_thresholds = stereo_voice_bandwidth_thresholds;
           music_bandwidth_thresholds = stereo_music_bandwidth_thresholds;
        } else {//ѡ����������
           voice_bandwidth_thresholds = mono_voice_bandwidth_thresholds;
           music_bandwidth_thresholds = mono_music_bandwidth_thresholds;
        }
        /*�����������voice_est�Ľ����opus_auto)���¼���������������   Interpolate bandwidth thresholds depending on voice estimation */
        for (i=0;i<8;i++)//��������Ϊ9807��1000��10807��1000��13017��1000��13614��2000������ֻ���������voice_est�Ľ���йأ�Ҳ����ֻ��Ԥ����ź����ͺ�Ӧ��ģʽ����
        {
           bandwidth_thresholds[i] = music_bandwidth_thresholds[i] + ((voice_est*voice_est*(voice_bandwidth_thresholds[i]-music_bandwidth_thresholds[i]))>>14);
        }
        do {
            int threshold, hysteresis;
            threshold = bandwidth_thresholds[2*(bandwidth-OPUS_BANDWIDTH_MEDIUMBAND)];//����=13614
            hysteresis = bandwidth_thresholds[2*(bandwidth-OPUS_BANDWIDTH_MEDIUMBAND)+1];//�ͺ�=2000
            if (!st->first)//0
            {
                if (st->auto_bandwidth >= bandwidth)
                    threshold -= hysteresis;//���޼�ȥ�ͺ�
                else
                    threshold += hysteresis;
            }
            if (equiv_rate >= threshold)
                break;
        } while (--bandwidth>OPUS_BANDWIDTH_NARROWBAND);
        st->bandwidth = st->auto_bandwidth = bandwidth;//=1103<8khz
        /* ��ֹ�κι��ɵ� SWB/FB��ֱ�� SILK ����ȫ�л��� WB ģʽ�����رտɱ� LP �˲���Prevents any transition to SWB/FB until the SILK layer has fully
           switched to WB mode and turned the variable LP filter off */
        if (!st->first && st->mode != MODE_CELT_ONLY && !st->silk_mode.inWBmodeWithoutVariableLP && st->bandwidth > OPUS_BANDWIDTH_WIDEBAND)
            st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    }

    if (st->bandwidth>st->max_bandwidth)
       st->bandwidth = st->max_bandwidth;

    if (st->user_bandwidth != OPUS_AUTO)
        st->bandwidth = st->user_bandwidth;

    /* �����ֹ�������ڲ���ȫ�� CBR/max ������ʹ�û��ģʽThis prevents us from using hybrid at unsafe CBR/max rates */
    if (st->mode != MODE_CELT_ONLY && max_rate < 15000)
    {
       st->bandwidth = IMIN(st->bandwidth, OPUS_BANDWIDTH_WIDEBAND);
    }

    /* ��ֹ Opus �ڸ��� Nyquist Ƶ�ʵ������ź����˷�λPrevents Opus from wasting bits on frequencies that are above
       the Nyquist rate of the input signal */
    if (st->Fs <= 24000 && st->bandwidth > OPUS_BANDWIDTH_SUPERWIDEBAND)
        st->bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND;
    if (st->Fs <= 16000 && st->bandwidth > OPUS_BANDWIDTH_WIDEBAND)
        st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    if (st->Fs <= 12000 && st->bandwidth > OPUS_BANDWIDTH_MEDIUMBAND)
        st->bandwidth = OPUS_BANDWIDTH_MEDIUMBAND;
    if (st->Fs <= 8000 && st->bandwidth > OPUS_BANDWIDTH_NARROWBAND)                                                      //Ϊ�˷����ο�˹�ز�������
        st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;//�������st->bandwidth=1101<4khzNB
#ifndef DISABLE_FLOAT_API
    /* Use detected bandwidth to reduce the encoded bandwidth. */
    if (st->detected_bandwidth && st->user_bandwidth == OPUS_AUTO)
    {
       int min_detected_bandwidth;
       /* Makes bandwidth detection more conservative just in case the detector
          gets it wrong when we could have coded a high bandwidth transparently.
          When operating in SILK/hybrid mode, we don't go below wideband to avoid
          more complicated switches that require redundancy. */
       if (equiv_rate <= 18000*st->stream_channels && st->mode == MODE_CELT_ONLY)
          min_detected_bandwidth = OPUS_BANDWIDTH_NARROWBAND;
       else if (equiv_rate <= 24000*st->stream_channels && st->mode == MODE_CELT_ONLY)
          min_detected_bandwidth = OPUS_BANDWIDTH_MEDIUMBAND;
       else if (equiv_rate <= 30000*st->stream_channels)
          min_detected_bandwidth = OPUS_BANDWIDTH_WIDEBAND;
       else if (equiv_rate <= 44000*st->stream_channels)
          min_detected_bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND;
       else
          min_detected_bandwidth = OPUS_BANDWIDTH_FULLBAND;

       st->detected_bandwidth = IMAX(st->detected_bandwidth, min_detected_bandwidth);
       st->bandwidth = IMIN(st->bandwidth, st->detected_bandwidth);
    }
#endif
    st->silk_mode.LBRR_coded = decide_fec(st->silk_mode.useInBandFEC, st->silk_mode.packetLossPercentage,st->silk_mode.LBRR_coded, st->mode, &st->bandwidth, equiv_rate);//�����Ƿ����LBRR֡ //������lbrr֡
    celt_encoder_ctl(celt_enc, OPUS_SET_LSB_DEPTH(lsb_depth));

    /* CELT ģʽ��֧����Ƶ�����ÿ�ƵCELT mode doesn't support mediumband, use wideband instead */
    if (st->mode == MODE_CELT_ONLY && st->bandwidth == OPUS_BANDWIDTH_MEDIUMBAND)
        st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    if (st->lfe)
       st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;

    curr_bandwidth = st->bandwidth;

    /* Chooses the appropriate mode for speechѡ����ʵ�����ģʽ ��Զ��Ҫ�������л������л��� CELT ģʽ����Ϊ���ʹһЩ����ʧЧ
       *NEVER* switch to/from CELT-only mode here as this will invalidate some assumptions */
    if (st->mode == MODE_SILK_ONLY && curr_bandwidth > OPUS_BANDWIDTH_WIDEBAND)//>8khz���ǻ��ģʽ
        st->mode = MODE_HYBRID;
    if (st->mode == MODE_HYBRID && curr_bandwidth <= OPUS_BANDWIDTH_WIDEBAND)//<8khz����ֻ��silkģʽ
        st->mode = MODE_SILK_ONLY;

    /* �����룬�޷��������60�������Ƶ֡���ڻ�ϻ�CELT-onlyģʽ�£����ܴ������20�������Ƶ֡,Ϊ���ܴ�����з�֡����Can't support higher than >60 ms frames, and >20 ms when in Hybrid or CELT-only modes */
    if ((frame_size > st->Fs/50 && (st->mode != MODE_SILK_ONLY)) || frame_size > 3*st->Fs/50)
    {
       int enc_frame_size;//����һ֡��֡��С
       int nb_frames;

       if (st->mode == MODE_SILK_ONLY)
       {
         if (frame_size == 2*st->Fs/25)  /* 80 ms -> 2x 40 ms    320*2*/
           enc_frame_size = st->Fs/25;
         else if (frame_size == 3*st->Fs/25)  /* 120 ms -> 2x 60 ms      320*3*/
           enc_frame_size = 3*st->Fs/50;
         else                            /* 100 ms -> 5x 20 ms      160 */
           enc_frame_size = st->Fs/50;
       }
       else
         enc_frame_size = st->Fs/50;//160
       nb_frames = frame_size/enc_frame_size;//=3
#ifndef DISABLE_FLOAT_API
       if (analysis_read_pos_bak!= -1)
       {
          st->analysis.read_pos = analysis_read_pos_bak;
          st->analysis.read_subframe = analysis_read_subframe_bak;
       }
#endif

       ret = encode_multiframe_packet(st, pcm, nb_frames, enc_frame_size, data,out_data_bytes, to_celt, lsb_depth, float_api);

       RESTORE_STACK;
       return ret;
    }
    /* һ���µ� SILK ����ĵ�һ֡For the first frame at a new SILK bandwidth */
    if (st->silk_bw_switch)//st->silk_bw_switch=0
    {
       redundancy = 1;
       celt_to_silk = 1;
       st->silk_bw_switch = 0;
       prefill=1;
    }

    /*������Ǿ���ʹ�� CELT����������֮ǰ����ʲô��������Ҫȷ���ر�����If we decided to go with CELT, make sure redundancy is off, no matter what
       we decided earlier. */
    if (st->mode == MODE_CELT_ONLY)
        redundancy = 0;
    if (redundancy)// redundancy = 0�����������ͼ��������ֽ�
    {
       redundancy_bytes = compute_redundancy_bytes(max_data_bytes, st->bitrate_bps, frame_rate, st->stream_channels);
       if (redundancy_bytes == 0)
          redundancy = 0;
    }
     /*printf("%d %d %d %d\n", st->bitrate_bps, st->stream_channels, st->mode, curr_bandwidth);*/
    bytes_target = IMIN(max_data_bytes-redundancy_bytes, st->bitrate_bps * frame_size / (st->Fs * 8)) - 1;//����Ŀ���ֽ�
                                                                                                   //��������һ���ֽ���ΪTOC�ֶ�Ԥ���ģ�����Ϊ�ڱ�����֮�������TOC�ֶ� 
    data += 1;
                                                                                                   //�����������ʼ����ÿһ֡�����ʼ�����������������綪��������ʱ֡���ǲ�����Ӱ���
    ec_enc_init(&enc, data, max_data_bytes-1);

    ALLOC(pcm_buf, (total_buffer+frame_size)*st->channels, opus_val16);
    OPUS_COPY(pcm_buf, &st->delay_buffer[(st->encoder_buffer-total_buffer)*st->channels], total_buffer*st->channels);    //delay_buffer�д������һ֡��4ms�����ݣ���һ֡ȫ��
                                                                                                                         //��������������music���ȷ����ͨƽ��Ƶ��
    if (st->mode == MODE_CELT_ONLY)
       hp_freq_smth1 = silk_LSHIFT( silk_lin2log( VARIABLE_HP_MIN_CUTOFF_HZ ), 8 );
    else
       hp_freq_smth1 = ((silk_encoder*)silk_enc)->state_Fxx[0].sCmn.variable_HP_smth1_Q15;                             //��ͨƽ������������һ֡�������ˮƽ�й�

    st->variable_HP_smth2_Q15 = silk_SMLAWB( st->variable_HP_smth2_Q15,hp_freq_smth1 - st->variable_HP_smth2_Q15, SILK_FIX_CONST( VARIABLE_HP_SMTH_COEF2, 16 ) ); //������һ֡�������ˮƽ����һ֡��ƽ���������и��²���
    /* convert from log scale to Hertz */
    cutoff_Hz = silk_log2lin( silk_RSHIFT( st->variable_HP_smth2_Q15, 8 ) );                            //��ƽ����������ΪƵ��
                                                                                                        //����ʹ��ģʽ������Ԥ�˲���music����ֻȥ��ֱ�����������������ڽ�ֹƵ��cutoff_Hz���µ��ź������෢��������ڸ�Ƶ��
    if (st->application == OPUS_APPLICATION_VOIP)
    {
       hp_cutoff(pcm, cutoff_Hz, &pcm_buf[total_buffer*st->channels], st->hp_mem, frame_size, st->channels, st->Fs, st->arch);//ȥ��Ƶ
    } else {
       dc_reject(pcm, 3, &pcm_buf[total_buffer*st->channels], st->hp_mem, frame_size, st->channels, st->Fs);//��ֱ��
    }
                                                                                                         //OPUS_APPLICATION_VOIP��OPUS_APPLICATION_AUDIOģʽ������������ϱ�����������ѡ��
#ifndef FIXED_POINT
    if (float_api)
    {
       opus_val32 sum;
       sum = celt_inner_prod(&pcm_buf[total_buffer*st->channels], &pcm_buf[total_buffer*st->channels], frame_size*st->channels, st->arch);
       /* This should filter out both NaNs and ridiculous signals that could
          cause NaNs further down. */
       if (!(sum < 1e9f) || celt_isnan(sum))
       {
          OPUS_CLEAR(&pcm_buf[total_buffer*st->channels], frame_size*st->channels);
          st->hp_mem[0] = st->hp_mem[1] = st->hp_mem[2] = st->hp_mem[3] = 0;
       }
    }
#endif


    /* SILK processing */
    HB_gain = Q15ONE;
    if (st->mode != MODE_CELT_ONLY)  //ģʽֻҪ����celt_only�ͽ��룬�������ȼ���silk���
    {
        opus_int32 total_bitRate, celt_rate;
        opus_int activity;
#ifdef FIXED_POINT
       const opus_int16 *pcm_silk;
#else
       VARDECL(opus_int16, pcm_silk);
       ALLOC(pcm_silk, st->channels*frame_size, opus_int16);
#endif

        activity = VAD_NO_DECISION;                                                                              //���û������
#ifndef DISABLE_FLOAT_API
        if( analysis_info.valid ) {
            /* Inform SILK about the Opus VAD decision */
            activity = ( analysis_info.activity_probability >= DTX_ACTIVITY_THRESHOLD );
        }
#endif

        /* ��λ�����SILK��CELTDistribute bits between SILK and CELT */
        total_bitRate = 8 * bytes_target * frame_rate;//8*66*16
        if( st->mode == MODE_HYBRID ) {
            /* Base rate for SILK */
            st->silk_mode.bitRate = compute_silk_rate_for_hybrid(total_bitRate,
                  curr_bandwidth, st->Fs == 50 * frame_size, st->use_vbr, st->silk_mode.LBRR_coded,
                  st->stream_channels);
            if (!st->energy_masking)
            {
               /* Increasingly attenuate high band when it gets allocated fewer bits */
               celt_rate = total_bitRate - st->silk_mode.bitRate;
               HB_gain = Q15ONE - SHR32(celt_exp2(-celt_rate * QCONST16(1.f/1024, 10)), 1);
            }
        } else {
            /* SILK gets all bits */
            st->silk_mode.bitRate = total_bitRate;                                                                //������λ��silk������
        }
        /* �ڱ�ЧӦsilkSurround masking for SILK   �ڱ�ЧӦû�м��㣬Ĭ��=0�򲻽���*/
        if (st->energy_masking && st->use_vbr && !st->lfe)                                                         //energy_masking=0Ĭ��ֵ
        {
           opus_val32 mask_sum=0;
           opus_val16 masking_depth;
           opus_int32 rate_offset;
           int c;
           int end = 17;
           opus_int16 srate = 16000;
           if (st->bandwidth == OPUS_BANDWIDTH_NARROWBAND)
           {
              end = 13;
              srate = 8000;
           } else if (st->bandwidth == OPUS_BANDWIDTH_MEDIUMBAND)
           {
              end = 15;
              srate = 12000;
           }
           for (c=0;c<st->channels;c++)
           {
              for(i=0;i<end;i++)
              {
                 opus_val16 mask;
                 mask = MAX16(MIN16(st->energy_masking[21*c+i],
                        QCONST16(.5f, DB_SHIFT)), -QCONST16(2.0f, DB_SHIFT));
                 if (mask > 0)
                    mask = HALF16(mask);
                 mask_sum += mask;
              }
           }
           /* Conservative rate reduction, we cut the masking in half */
           masking_depth = mask_sum / end*st->channels;
           masking_depth += QCONST16(.2f, DB_SHIFT);
           rate_offset = (opus_int32)PSHR32(MULT16_16(srate, masking_depth), DB_SHIFT);
           rate_offset = MAX32(rate_offset, -2*st->silk_mode.bitRate/3);
           /* Split the rate change between the SILK and CELT part for hybrid. */
           if (st->bandwidth==OPUS_BANDWIDTH_SUPERWIDEBAND || st->bandwidth==OPUS_BANDWIDTH_FULLBAND)
              st->silk_mode.bitRate += 3*rate_offset/5;
           else
              st->silk_mode.bitRate += rate_offset;
        }

        st->silk_mode.payloadSize_ms = 1000 * frame_size / st->Fs;                                           //��framesizeת��Ϊms��������������silk_mode�ṹ��
        st->silk_mode.nChannelsAPI = st->channels;
        st->silk_mode.nChannelsInternal = st->stream_channels;
        if (curr_bandwidth == OPUS_BANDWIDTH_NARROWBAND)                                                     //��ǰ֡�Ĵ�����������������ʣ�խ����Ӧ8000���д����Ӧ12000�������Ӧ16000
            st->silk_mode.desiredInternalSampleRate = 8000;//�����
        else 
            if (curr_bandwidth == OPUS_BANDWIDTH_MEDIUMBAND)
            st->silk_mode.desiredInternalSampleRate = 12000;
            else 
            {
            celt_assert( st->mode == MODE_HYBRID || curr_bandwidth == OPUS_BANDWIDTH_WIDEBAND );
            st->silk_mode.desiredInternalSampleRate = 16000;
            }
        if( st->mode == MODE_HYBRID ) {
            /* �ڻ��ģʽ�£���Ҫ����ͱ�����ʱ��������͡�Don't allow bandwidth reduction at lowest bitrates in hybrid mode ���ģʽ������celt��ԭ����Ϊ��ǰ��Ƶ�������֣���˽���С�Ĳ���������Ϊ16000*/
            st->silk_mode.minInternalSampleRate = 16000;
        } else {
            st->silk_mode.minInternalSampleRate = 8000;                                                       //���ݱ���ģʽ�趨��С�ڲ������ʺ�����ڲ�������
        }                                                                                                     //ֻʹ��silkģʽʱ����С�ڲ������ʾ�����Ϊ8000

        st->silk_mode.maxInternalSampleRate = 16000;                                                         //ֻҪ����celtģʽ����ڲ������ʾ�����Ϊ16000
        if (st->mode == MODE_SILK_ONLY)
        {
           opus_int32 effective_max_rate = max_rate;
           if (frame_rate > 50)
              effective_max_rate = effective_max_rate*2/3;
           if (effective_max_rate < 8000)
           {
              st->silk_mode.maxInternalSampleRate = 12000;
              st->silk_mode.desiredInternalSampleRate = IMIN(12000, st->silk_mode.desiredInternalSampleRate);
           }
           if (effective_max_rate < 7000)
           {
              st->silk_mode.maxInternalSampleRate = 8000;
              st->silk_mode.desiredInternalSampleRate = IMIN(8000, st->silk_mode.desiredInternalSampleRate);
           }
        }

        st->silk_mode.useCBR = !st->use_vbr;                                                                 //cbr��vbr����ͬʱ����

        /* Call SILK encoder for the low band */

        /* ����SILK�������Ƶ���ݣ����ǵ�ToC��Type of Content��֡ͷ��Ϣ�������ֽںͿ�ѡ���������ֽڣ����ܴﵽ����������Max bits for SILK, counting ToC, redundancy bytes, and optionally redundancy. */
        st->silk_mode.maxBits = (max_data_bytes-1)*8;//=10200                                                //silk֡������ֽ���
        if (redundancy && redundancy_bytes >= 2)
        {
           /* Counting 1 bit for redundancy position and 20 bits for flag+size (only for hybrid). */
           st->silk_mode.maxBits -= redundancy_bytes*8 + 1;
           if (st->mode == MODE_HYBRID)
              st->silk_mode.maxBits -= 20;
        }
        if (st->silk_mode.useCBR)
        {
           if (st->mode == MODE_HYBRID)
           {
              st->silk_mode.maxBits = IMIN(st->silk_mode.maxBits, st->silk_mode.bitRate * frame_size / st->Fs);
           }
        } else {
           /* Constrained VBR. */
           if (st->mode == MODE_HYBRID)
           {
              /* Compute SILK bitrate corresponding to the max total bits available */
              opus_int32 maxBitRate = compute_silk_rate_for_hybrid(st->silk_mode.maxBits*st->Fs / frame_size,
                    curr_bandwidth, st->Fs == 50 * frame_size, st->use_vbr, st->silk_mode.LBRR_coded,
                    st->stream_channels);
              st->silk_mode.maxBits = maxBitRate * frame_size / st->Fs;
           }
        }
        if (prefill)//=0
        {
            opus_int32 zero=0;
            int prefill_offset;
            /* ��SILKǰ���Ŀ�ʼ��������Ϊƽ�����ɣ�������������Ա����ϡ�ȷ�е�λ����������Ҫ����������CELT֡���ʱ����Ƶ�������κΡ���϶������������ǿ��Ը���st->delay_buffer����Ϊ��������д֮ǰΨһʹ��������tmp_prefill[]����ʹ��ʹ�õĲ���Ҳֻ�й������򣬶��Ƿ��͵��������������Ĳ��֡�Use a smooth onset for the SILK prefill to avoid the encoder trying to encode
               a discontinuity. The exact location is what we need to avoid leaving any "gap"
               in the audio when mixing with the redundant CELT frame. Here we can afford to
               overwrite st->delay_buffer because the only thing that uses it before it gets
               rewritten is tmp_prefill[] and even then only the part after the ramp really
               gets used (rather than sent to the encoder and discarded) */
            prefill_offset = st->channels*(st->encoder_buffer-st->delay_compensation-st->Fs/400);
            gain_fade(st->delay_buffer+prefill_offset, st->delay_buffer+prefill_offset,
                  0, Q15ONE, celt_mode->overlap, st->Fs/400, st->channels, celt_mode->window, st->Fs);
            OPUS_CLEAR(st->delay_buffer, prefill_offset);
#ifdef FIXED_POINT
            pcm_silk = st->delay_buffer;
#else
            for (i=0;i<st->encoder_buffer*st->channels;i++)
                pcm_silk[i] = FLOAT2INT16(st->delay_buffer[i]);
#endif
            //silk����
            silk_Encode( silk_enc, &st->silk_mode, pcm_silk, st->encoder_buffer, NULL, &zero, 1, activity );
        }

#ifdef FIXED_POINT
        pcm_silk = pcm_buf+total_buffer*st->channels;//��ַ��������32λ
#else
        for (i=0;i<frame_size*st->channels;i++)
            pcm_silk[i] = FLOAT2INT16(pcm_buf[total_buffer*st->channels + i]);
#endif
//short����pcm���ݴ洢�׵�ַpcm_silk
 //frame_size:480 8kHz 60ms֡��
 //nByte:������ֽ���
 //activity��ָʾ�����������
        ret = silk_Encode( silk_enc, &st->silk_mode, pcm_silk, frame_size, &enc, &nBytes, 0, activity );
        if( ret ) {
            /*fprintf (stderr, "SILK encode error: %d\n", ret);*/
            /* Handle error */
           RESTORE_STACK;
           return OPUS_INTERNAL_ERROR;
        }
        /* Extract SILK internal bandwidth for signaling in first byte */
        if( st->mode == MODE_SILK_ONLY ) {
            if( st->silk_mode.internalSampleRate == 8000 ) {
               curr_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//�����
            } else if( st->silk_mode.internalSampleRate == 12000 ) {
               curr_bandwidth = OPUS_BANDWIDTH_MEDIUMBAND;
            } else if( st->silk_mode.internalSampleRate == 16000 ) {
               curr_bandwidth = OPUS_BANDWIDTH_WIDEBAND;
            }
        } else {
            celt_assert( st->silk_mode.internalSampleRate == 16000 );
        }

        st->silk_mode.opusCanSwitch = st->silk_mode.switchReady && !st->nonfinal_frame;

        if (nBytes==0)
        {
           st->rangeFinal = 0;
           data[-1] = gen_toc(st->mode, st->Fs/frame_size, curr_bandwidth, st->stream_channels);
           RESTORE_STACK;
           return 1;
        }

        /* FIXME: How do we allocate the redundancy for CBR? */
        if (st->silk_mode.opusCanSwitch)
        {
           redundancy_bytes = compute_redundancy_bytes(max_data_bytes, st->bitrate_bps, frame_rate, st->stream_channels);
           redundancy = (redundancy_bytes != 0);
           celt_to_silk = 0;
           st->silk_bw_switch = 1;
        }
    }

    /* CELT processing */
    {
        int endband=21;

        switch(curr_bandwidth)
        {
            case OPUS_BANDWIDTH_NARROWBAND:
                endband = 13;//
                break;
            case OPUS_BANDWIDTH_MEDIUMBAND:
            case OPUS_BANDWIDTH_WIDEBAND:
                endband = 17;
                break;
            case OPUS_BANDWIDTH_SUPERWIDEBAND:
                endband = 19;
                break;
            case OPUS_BANDWIDTH_FULLBAND:
                endband = 21;
                break;
        }
        celt_encoder_ctl(celt_enc, CELT_SET_END_BAND(endband));//
        celt_encoder_ctl(celt_enc, CELT_SET_CHANNELS(st->stream_channels));//
    }
    celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(OPUS_BITRATE_MAX));//
    if (st->mode != MODE_SILK_ONLY)//����
    {
        printf("opus_encoder.c 2041\n");/////////////////////////////////////////////
        opus_val32 celt_pred=2;
        celt_encoder_ctl(celt_enc, OPUS_SET_VBR(0));
        /* We may still decide to disable prediction later */
        if (st->silk_mode.reducedDependency)
           celt_pred = 0;
        celt_encoder_ctl(celt_enc, CELT_SET_PREDICTION(celt_pred));

        if (st->mode == MODE_HYBRID)
        {
            if( st->use_vbr ) {
                celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(st->bitrate_bps-st->silk_mode.bitRate));
                celt_encoder_ctl(celt_enc, OPUS_SET_VBR_CONSTRAINT(0));
            }
        } else {
            if (st->use_vbr)
            {
                celt_encoder_ctl(celt_enc, OPUS_SET_VBR(1));
                celt_encoder_ctl(celt_enc, OPUS_SET_VBR_CONSTRAINT(st->vbr_constraint));
                celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(st->bitrate_bps));
            }
        }
    }

    ALLOC(tmp_prefill, st->channels*st->Fs/400, opus_val16);//
    if (st->mode != MODE_SILK_ONLY && st->mode != st->prev_mode && st->prev_mode > 0)//����
    {
        printf("opus_encoder.c 2068\n");
       OPUS_COPY(tmp_prefill, &st->delay_buffer[(st->encoder_buffer-total_buffer-st->Fs/400)*st->channels], st->channels*st->Fs/400);
    }

    if (st->channels*(st->encoder_buffer-(frame_size+total_buffer)) > 0)//����
    {
        printf("opus_encoder.c 2074\n");
       OPUS_MOVE(st->delay_buffer, &st->delay_buffer[st->channels*frame_size], st->channels*(st->encoder_buffer-frame_size-total_buffer));
       OPUS_COPY(&st->delay_buffer[st->channels*(st->encoder_buffer-frame_size-total_buffer)],
             &pcm_buf[0],
             (frame_size+total_buffer)*st->channels);
    } else {
       OPUS_COPY(st->delay_buffer, &pcm_buf[(frame_size+total_buffer-st->encoder_buffer)*st->channels], st->encoder_buffer*st->channels);//����һ֡��ǰtotal_buffer��Ԫ�ؼӵ���һ֡ǰ��
    }
    /* gain_fade() and stereo_fade() need to be after the buffer copying
       because we don't want any of this to affect the SILK part */
    if( st->prev_HB_gain < Q15ONE || HB_gain < Q15ONE ) {//����
        printf("opus_encoder.c 2085\n");
       gain_fade(pcm_buf, pcm_buf,
             st->prev_HB_gain, HB_gain, celt_mode->overlap, frame_size, st->channels, celt_mode->window, st->Fs);
    }
    st->prev_HB_gain = HB_gain;//
    if (st->mode != MODE_HYBRID || st->stream_channels==1)
    {
       if (equiv_rate > 32000)
          st->silk_mode.stereoWidth_Q14 = 16384;
       else if (equiv_rate < 16000)
          st->silk_mode.stereoWidth_Q14 = 0;//
       else
          st->silk_mode.stereoWidth_Q14 = 16384 - 2048*(opus_int32)(32000-equiv_rate)/(equiv_rate-14000);
    }
    if( !st->energy_masking && st->channels == 2 ) {//����
        printf("opus_encoder.c 2100\n");
        /* Apply stereo width reduction (at low bitrates) */
        if( st->hybrid_stereo_width_Q14 < (1 << 14) || st->silk_mode.stereoWidth_Q14 < (1 << 14) ) {
            opus_val16 g1, g2;
            g1 = st->hybrid_stereo_width_Q14;
            g2 = (opus_val16)(st->silk_mode.stereoWidth_Q14);
#ifdef FIXED_POINT
            g1 = g1==16384 ? Q15ONE : SHL16(g1,1);
            g2 = g2==16384 ? Q15ONE : SHL16(g2,1);
#else
            g1 *= (1.f/16384);
            g2 *= (1.f/16384);
#endif
            stereo_fade(pcm_buf, pcm_buf, g1, g2, celt_mode->overlap,
                  frame_size, st->channels, celt_mode->window, st->Fs);
            st->hybrid_stereo_width_Q14 = st->silk_mode.stereoWidth_Q14;
        }
    }

    if ( st->mode != MODE_CELT_ONLY && ec_tell(&enc)+17+20*(st->mode == MODE_HYBRID) <= 8*(max_data_bytes-1))
    {
        /* For SILK mode, the redundancy is inferred from the length */
        if (st->mode == MODE_HYBRID)//����
        {
            printf("opus_encoder.c 2124\n");
            ec_enc_bit_logp(&enc, redundancy, 12);
        }
        if (redundancy)//����
        {
            printf("opus_encoder.c 2129\n");
            int max_redundancy;
            ec_enc_bit_logp(&enc, celt_to_silk, 1);
            if (st->mode == MODE_HYBRID)
            {
               /* Reserve the 8 bits needed for the redundancy length,
                  and at least a few bits for CELT if possible */
               max_redundancy = (max_data_bytes-1)-((ec_tell(&enc)+8+3+7)>>3);
            }
            else
               max_redundancy = (max_data_bytes-1)-((ec_tell(&enc)+7)>>3);
            /* Target the same bit-rate for redundancy as for the rest,
               up to a max of 257 bytes */
            redundancy_bytes = IMIN(max_redundancy, redundancy_bytes);
            redundancy_bytes = IMIN(257, IMAX(2, redundancy_bytes));
            if (st->mode == MODE_HYBRID)
                ec_enc_uint(&enc, redundancy_bytes-2, 256);
        }
    } else {
        redundancy = 0;//ֻ���������
    }

    if (!redundancy)//
    {
       st->silk_bw_switch = 0;//
       redundancy_bytes = 0;//
    }
    if (st->mode != MODE_CELT_ONLY)start_band=17;//

    if (st->mode == MODE_SILK_ONLY)
    {
        ret = (ec_tell(&enc)+7)>>3;//
        ec_enc_done(&enc);//
        nb_compr_bytes = ret;//
    } else {//����
       nb_compr_bytes = (max_data_bytes-1)-redundancy_bytes;
       ec_enc_shrink(&enc, nb_compr_bytes);
    }

#ifndef DISABLE_FLOAT_API
    if (redundancy || st->mode != MODE_SILK_ONLY)
       celt_encoder_ctl(celt_enc, CELT_SET_ANALYSIS(&analysis_info));
#endif
    if (st->mode == MODE_HYBRID) {//����
        printf("opus_encoder.c 2173\n");///////////////////////////////////////////////////////////
       SILKInfo info;
       info.signalType = st->silk_mode.signalType;
       info.offset = st->silk_mode.offset;
       celt_encoder_ctl(celt_enc, CELT_SET_SILK_INFO(&info));
    } else {
       celt_encoder_ctl(celt_enc, CELT_SET_SILK_INFO((SILKInfo*)NULL));//
    }

    /* 5 ms redundant frame for CELT->SILK */
    if (redundancy && celt_to_silk)//����
    {
        printf("opus_encoder.c 2185\n");///////////////////////////////////////////////////////////////
        int err;
        celt_encoder_ctl(celt_enc, CELT_SET_START_BAND(0));
        celt_encoder_ctl(celt_enc, OPUS_SET_VBR(0));
        celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(OPUS_BITRATE_MAX));
        err = celt_encode_with_ec(celt_enc, pcm_buf, st->Fs/200, data+nb_compr_bytes, redundancy_bytes, NULL);
        if (err < 0)
        {
           RESTORE_STACK;
           return OPUS_INTERNAL_ERROR;
        }
        celt_encoder_ctl(celt_enc, OPUS_GET_FINAL_RANGE(&redundant_rng));
        celt_encoder_ctl(celt_enc, OPUS_RESET_STATE);
        printf("opus_encoder.c celttosilk\n");//////////////////////////////////////////////////////
    }

    celt_encoder_ctl(celt_enc, CELT_SET_START_BAND(start_band));//

    if (st->mode != MODE_SILK_ONLY)//����
    {
        printf("opus_encoder.c 2205\n");////////////////////////////////////////////////////////////
        if (st->mode != st->prev_mode && st->prev_mode > 0)
        {
           unsigned char dummy[2];
           celt_encoder_ctl(celt_enc, OPUS_RESET_STATE);

           /* Prefilling */
           celt_encode_with_ec(celt_enc, tmp_prefill, st->Fs/400, dummy, 2, NULL);
           celt_encoder_ctl(celt_enc, CELT_SET_PREDICTION(0));
        }
        /* If false, we already busted the budget and we'll end up with a "PLC frame" */
        if (ec_tell(&enc) <= 8*nb_compr_bytes)
        {
           /* Set the bitrate again if it was overridden in the redundancy code above*/
           if (redundancy && celt_to_silk && st->mode==MODE_HYBRID && st->use_vbr)
              celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(st->bitrate_bps-st->silk_mode.bitRate));
           celt_encoder_ctl(celt_enc, OPUS_SET_VBR(st->use_vbr));
           ret = celt_encode_with_ec(celt_enc, pcm_buf, frame_size, NULL, nb_compr_bytes, &enc);
           if (ret < 0)
           {
              RESTORE_STACK;
              return OPUS_INTERNAL_ERROR;
           }
           /* Put CELT->SILK redundancy data in the right place. */
           if (redundancy && celt_to_silk && st->mode==MODE_HYBRID && st->use_vbr)
           {
              OPUS_MOVE(data+ret, data+nb_compr_bytes, redundancy_bytes);
              nb_compr_bytes = nb_compr_bytes+redundancy_bytes;
           }
        }
    }

    /* 5 ms redundant frame for SILK->CELT */
    if (redundancy && !celt_to_silk)//����
    {
        printf("opus_encoder.c 2240\n");////////////////////////////
        int err;
        unsigned char dummy[2];
        int N2, N4;
        N2 = st->Fs/200;
        N4 = st->Fs/400;
        celt_encoder_ctl(celt_enc, OPUS_RESET_STATE);
        celt_encoder_ctl(celt_enc, CELT_SET_START_BAND(0));
        celt_encoder_ctl(celt_enc, CELT_SET_PREDICTION(0));
        celt_encoder_ctl(celt_enc, OPUS_SET_VBR(0));
        celt_encoder_ctl(celt_enc, OPUS_SET_BITRATE(OPUS_BITRATE_MAX));

        if (st->mode == MODE_HYBRID)
        {
           /* Shrink packet to what the encoder actually used. */
           nb_compr_bytes = ret;
           ec_enc_shrink(&enc, nb_compr_bytes);
        }
        /* NOTE: We could speed this up slightly (at the expense of code size) by just adding a function that prefills the buffer */
        celt_encode_with_ec(celt_enc, pcm_buf+st->channels*(frame_size-N2-N4), N4, dummy, 2, NULL);

        err = celt_encode_with_ec(celt_enc, pcm_buf+st->channels*(frame_size-N2), N2, data+nb_compr_bytes, redundancy_bytes, NULL);
        if (err < 0)
        {
           RESTORE_STACK;
           return OPUS_INTERNAL_ERROR;
        }
        celt_encoder_ctl(celt_enc, OPUS_GET_FINAL_RANGE(&redundant_rng));
    }

    

    /* Signalling the mode in the first byte */
    //����silk_Encode���������TOC�ֶ�
    data--;//
    data[0] = gen_toc(st->mode, st->Fs/frame_size, curr_bandwidth, st->stream_channels);                  //data�д洢���� TOC �ֶκͳ�����Ϣ�����ֵ

    st->rangeFinal = enc.rng ^ redundant_rng;

    if (to_celt)
        st->prev_mode = MODE_CELT_ONLY;
    else
        st->prev_mode = st->mode;
    st->prev_channels = st->stream_channels;
    st->prev_framesize = frame_size;

    st->first = 0;

    /* DTX decision */
#ifndef DISABLE_FLOAT_API
    if (st->use_dtx && (analysis_info.valid || is_silence))
    {
       /* Mark the frame active if Silk considers it active */
       if(st->mode != MODE_CELT_ONLY && st->silk_mode.signalType != TYPE_NO_VOICE_ACTIVITY)
       {
          analysis_info.activity_probability = 1.0f;
       }

       if (decide_dtx_mode(analysis_info.activity_probability, &st->nb_no_activity_frames,
             st->peak_signal_energy, pcm, frame_size, st->channels, is_silence, st->arch))
       {
          st->rangeFinal = 0;
          data[0] = gen_toc(st->mode, st->Fs/frame_size, curr_bandwidth, st->stream_channels);
          RESTORE_STACK;
          return 1;
       }
    }
#endif

    /* In the unlikely case that the SILK encoder busted its target, tell
       the decoder to call the PLC */
    if (ec_tell(&enc) > (max_data_bytes-1)*8)//����
    {
       if (max_data_bytes < 2)
       {
          RESTORE_STACK;
          return OPUS_BUFFER_TOO_SMALL;
       }
       data[1] = 0;
       ret = 1;
       st->rangeFinal = 0;
    } else if (st->mode==MODE_SILK_ONLY&&!redundancy)//
    {
       /*When in LPC only mode it's perfectly
         reasonable to strip off trailing zero bytes as
         the required range decoder behavior is to
         fill these in. This can't be done when the MDCT
         modes are used because the decoder needs to know
         the actual length for allocation purposes.*/
       while(ret>2&&data[ret]==0)ret--;
    }
    /* Count ToC and redundancy */
    ret += 1+redundancy_bytes;//
    if (!st->use_vbr)//����
    {
       if (opus_packet_pad(data, ret, max_data_bytes) != OPUS_OK)
       {
          RESTORE_STACK;
          return OPUS_INTERNAL_ERROR;
       }
       ret = max_data_bytes;//���������vbr��ÿ֡�������ֽ�ѹ��
    }
    RESTORE_STACK;
    return ret;
}

#ifdef FIXED_POINT

#ifndef DISABLE_FLOAT_API
opus_int32 opus_encode_float(OpusEncoder *st, const float *pcm, int analysis_frame_size,
      unsigned char *data, opus_int32 max_data_bytes)
{
   int i, ret;
   int frame_size;
   VARDECL(opus_int16, in);
   ALLOC_STACK;

   frame_size = frame_size_select(analysis_frame_size, st->variable_duration, st->Fs);
   if (frame_size <= 0)
   {
      RESTORE_STACK;
      return OPUS_BAD_ARG;
   }
   ALLOC(in, frame_size*st->channels, opus_int16);

   for (i=0;i<frame_size*st->channels;i++)
      in[i] = FLOAT2INT16(pcm[i]);
   ret = opus_encode_native(st, in, frame_size, data, max_data_bytes, 16,
                            pcm, analysis_frame_size, 0, -2, st->channels, downmix_float, 1);
   RESTORE_STACK;
   return ret;
}
#endif
//����opus_encode_native���ý����Ӧ���Ǳ������Ԫ����
opus_int32 opus_encode(OpusEncoder *st, const opus_int16 *pcm, int analysis_frame_size,
                unsigned char *data, opus_int32 out_data_bytes)
{
   int frame_size;
   frame_size = frame_size_select(analysis_frame_size, st->variable_duration, st->Fs);//analysis_frame_size���û����õ�֡��С��variable_duration�����ص�ǰ֡���ݵ�ʱ��
                                                                                      //Ŀ�����ж����õ�֡��С�Ƿ������2.5ms��120ms֮�䣩
                                                                                      //���variable_duration����Ĭ�ϲ���������Ҫʹvariable_duration��analysis_frame_sizeƥ��
   return opus_encode_native(st, pcm, frame_size, data, out_data_bytes, 16,pcm, analysis_frame_size, 0, -2, st->channels, downmix_int, 0);//���뺯��
}

#else
opus_int32 opus_encode(OpusEncoder *st, const opus_int16 *pcm, int analysis_frame_size,
      unsigned char *data, opus_int32 max_data_bytes)
{
   int i, ret;
   int frame_size;
   VARDECL(float, in);
   ALLOC_STACK;

   frame_size = frame_size_select(analysis_frame_size, st->variable_duration, st->Fs);
   if (frame_size <= 0)
   {
      RESTORE_STACK;
      return OPUS_BAD_ARG;
   }
   ALLOC(in, frame_size*st->channels, float);

   for (i=0;i<frame_size*st->channels;i++)
      in[i] = (1.0f/32768)*pcm[i];
   //in:�����ʽ������������ݣ�frame_size:�Բ����������֡����data:�������ֽ����ŵĵ�ַ��
//max_data_bytes:�����ֽ������󳤶ȣ�lsb_depth:least significant bit �����Чλ��16
//pcm:short����Ƶ���ݣ�analysis_frame_size:����֡��20ms��320��
//C1:0 C2:-2, ����ͨ����st->channels, downmix_int���²���������ַ���ڣ�
//���һ������ָʾ����API
   ret = opus_encode_native(st, in, frame_size, data, max_data_bytes, 16,
                            pcm, analysis_frame_size, 0, -2, st->channels, downmix_int, 0);
   RESTORE_STACK;
   return ret;
}
opus_int32 opus_encode_float(OpusEncoder *st, const float *pcm, int analysis_frame_size,
                      unsigned char *data, opus_int32 out_data_bytes)
{
   int frame_size;
   frame_size = frame_size_select(analysis_frame_size, st->variable_duration, st->Fs);
   return opus_encode_native(st, pcm, frame_size, data, out_data_bytes, 24,
                             pcm, analysis_frame_size, 0, -2, st->channels, downmix_float, 1);
}
#endif


int opus_encoder_ctl(OpusEncoder *st, int request, ...)
{
    int ret;
    CELTEncoder *celt_enc;
    va_list ap;

    ret = OPUS_OK;
    va_start(ap, request);

    celt_enc = (CELTEncoder*)((char*)st+st->celt_enc_offset);

    switch (request)
    {
        case OPUS_SET_APPLICATION_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (   (value != OPUS_APPLICATION_VOIP && value != OPUS_APPLICATION_AUDIO
                 && value != OPUS_APPLICATION_RESTRICTED_LOWDELAY)
               || (!st->first && st->application != value))
            {
               ret = OPUS_BAD_ARG;
               break;
            }
            st->application = value;
#ifndef DISABLE_FLOAT_API
            st->analysis.application = value;
#endif
        }
        break;
        case OPUS_GET_APPLICATION_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->application;
        }
        break;
        case OPUS_SET_BITRATE_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value != OPUS_AUTO && value != OPUS_BITRATE_MAX)
            {
                if (value <= 0)
                    goto bad_arg;
                else if (value <= 500)
                    value = 500;
                else if (value > (opus_int32)300000*st->channels)
                    value = (opus_int32)300000*st->channels;
            }
            st->user_bitrate_bps = value;
        }
        break;
        case OPUS_GET_BITRATE_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = user_bitrate_to_bitrate(st, st->prev_framesize, 1276);
        }
        break;
        case OPUS_SET_FORCE_CHANNELS_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if((value<1 || value>st->channels) && value != OPUS_AUTO)
            {
               goto bad_arg;
            }
            st->force_channels = value;
        }
        break;
        case OPUS_GET_FORCE_CHANNELS_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->force_channels;
        }
        break;
        case OPUS_SET_MAX_BANDWIDTH_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value < OPUS_BANDWIDTH_NARROWBAND || value > OPUS_BANDWIDTH_FULLBAND)
            {
               goto bad_arg;
            }
            st->max_bandwidth = value;
            if (st->max_bandwidth == OPUS_BANDWIDTH_NARROWBAND) {
                st->silk_mode.maxInternalSampleRate = 8000;
            } else if (st->max_bandwidth == OPUS_BANDWIDTH_MEDIUMBAND) {
                st->silk_mode.maxInternalSampleRate = 12000;
            } else {
                st->silk_mode.maxInternalSampleRate = 16000;
            }
        }
        break;
        case OPUS_GET_MAX_BANDWIDTH_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->max_bandwidth;
        }
        break;
        case OPUS_SET_BANDWIDTH_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if ((value < OPUS_BANDWIDTH_NARROWBAND || value > OPUS_BANDWIDTH_FULLBAND) && value != OPUS_AUTO)
            {
               goto bad_arg;
            }
            st->user_bandwidth = value;
            if (st->user_bandwidth == OPUS_BANDWIDTH_NARROWBAND) {
                st->silk_mode.maxInternalSampleRate = 8000;
            } else if (st->user_bandwidth == OPUS_BANDWIDTH_MEDIUMBAND) {
                st->silk_mode.maxInternalSampleRate = 12000;
            } else {
                st->silk_mode.maxInternalSampleRate = 16000;
            }
        }
        break;
        case OPUS_GET_BANDWIDTH_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->bandwidth;
        }
        break;
        case OPUS_SET_DTX_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>1)
            {
               goto bad_arg;
            }
            st->use_dtx = value;
        }
        break;
        case OPUS_GET_DTX_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->use_dtx;
        }
        break;
        case OPUS_SET_COMPLEXITY_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>10)
            {
               goto bad_arg;
            }
            st->silk_mode.complexity = value;
            celt_encoder_ctl(celt_enc, OPUS_SET_COMPLEXITY(value));
        }
        break;
        case OPUS_GET_COMPLEXITY_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->silk_mode.complexity;
        }
        break;
        case OPUS_SET_INBAND_FEC_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>1)
            {
               goto bad_arg;
            }
            st->silk_mode.useInBandFEC = value;
        }
        break;
        case OPUS_GET_INBAND_FEC_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->silk_mode.useInBandFEC;
        }
        break;
        case OPUS_SET_PACKET_LOSS_PERC_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value < 0 || value > 100)
            {
               goto bad_arg;
            }
            st->silk_mode.packetLossPercentage = value;
            celt_encoder_ctl(celt_enc, OPUS_SET_PACKET_LOSS_PERC(value));
        }
        break;
        case OPUS_GET_PACKET_LOSS_PERC_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->silk_mode.packetLossPercentage;
        }
        break;
        case OPUS_SET_VBR_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>1)
            {
               goto bad_arg;
            }
            st->use_vbr = value;
            st->silk_mode.useCBR = 1-value;
        }
        break;
        case OPUS_GET_VBR_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->use_vbr;
        }
        break;
        case OPUS_SET_VOICE_RATIO_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value<-1 || value>100)
            {
               goto bad_arg;
            }
            st->voice_ratio = value;
        }
        break;
        case OPUS_GET_VOICE_RATIO_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->voice_ratio;
        }
        break;
        case OPUS_SET_VBR_CONSTRAINT_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>1)
            {
               goto bad_arg;
            }
            st->vbr_constraint = value;
        }
        break;
        case OPUS_GET_VBR_CONSTRAINT_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->vbr_constraint;
        }
        break;
        case OPUS_SET_SIGNAL_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value!=OPUS_AUTO && value!=OPUS_SIGNAL_VOICE && value!=OPUS_SIGNAL_MUSIC)
            {
               goto bad_arg;
            }
            st->signal_type = value;
        }
        break;
        case OPUS_GET_SIGNAL_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->signal_type;
        }
        break;
        case OPUS_GET_LOOKAHEAD_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->Fs/400;
            if (st->application != OPUS_APPLICATION_RESTRICTED_LOWDELAY)
                *value += st->delay_compensation;
        }
        break;
        case OPUS_GET_SAMPLE_RATE_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->Fs;
        }
        break;
        case OPUS_GET_FINAL_RANGE_REQUEST:
        {
            opus_uint32 *value = va_arg(ap, opus_uint32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->rangeFinal;
        }
        break;
        case OPUS_SET_LSB_DEPTH_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value<8 || value>24)
            {
               goto bad_arg;
            }
            st->lsb_depth=value;
        }
        break;
        case OPUS_GET_LSB_DEPTH_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->lsb_depth;
        }
        break;
        case OPUS_SET_EXPERT_FRAME_DURATION_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if (value != OPUS_FRAMESIZE_ARG    && value != OPUS_FRAMESIZE_2_5_MS &&
                value != OPUS_FRAMESIZE_5_MS   && value != OPUS_FRAMESIZE_10_MS  &&
                value != OPUS_FRAMESIZE_20_MS  && value != OPUS_FRAMESIZE_40_MS  &&
                value != OPUS_FRAMESIZE_60_MS  && value != OPUS_FRAMESIZE_80_MS  &&
                value != OPUS_FRAMESIZE_100_MS && value != OPUS_FRAMESIZE_120_MS)
            {
               goto bad_arg;
            }
            st->variable_duration = value;
            celt_encoder_ctl(celt_enc, OPUS_SET_EXPERT_FRAME_DURATION(value));
        }
        break;
        case OPUS_GET_EXPERT_FRAME_DURATION_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            *value = st->variable_duration;
        }
        break;
        case OPUS_SET_PREDICTION_DISABLED_REQUEST:
        {
           opus_int32 value = va_arg(ap, opus_int32);
           if (value > 1 || value < 0)
              goto bad_arg;
           st->silk_mode.reducedDependency = value;
        }
        break;
        case OPUS_GET_PREDICTION_DISABLED_REQUEST:
        {
           opus_int32 *value = va_arg(ap, opus_int32*);
           if (!value)
              goto bad_arg;
           *value = st->silk_mode.reducedDependency;
        }
        break;
        case OPUS_SET_PHASE_INVERSION_DISABLED_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if(value<0 || value>1)
            {
               goto bad_arg;
            }
            celt_encoder_ctl(celt_enc, OPUS_SET_PHASE_INVERSION_DISABLED(value));
        }
        break;
        case OPUS_GET_PHASE_INVERSION_DISABLED_REQUEST:
        {
            opus_int32 *value = va_arg(ap, opus_int32*);
            if (!value)
            {
               goto bad_arg;
            }
            celt_encoder_ctl(celt_enc, OPUS_GET_PHASE_INVERSION_DISABLED(value));
        }
        break;
        case OPUS_RESET_STATE:
        {
           void *silk_enc;
           silk_EncControlStruct dummy;
           char *start;
           silk_enc = (char*)st+st->silk_enc_offset;
#ifndef DISABLE_FLOAT_API
           tonality_analysis_reset(&st->analysis);
#endif

           start = (char*)&st->OPUS_ENCODER_RESET_START;
           OPUS_CLEAR(start, sizeof(OpusEncoder) - (start - (char*)st));

           celt_encoder_ctl(celt_enc, OPUS_RESET_STATE);
           silk_InitEncoder( silk_enc, st->arch, &dummy );
           st->stream_channels = st->channels;
           st->hybrid_stereo_width_Q14 = 1 << 14;
           st->prev_HB_gain = Q15ONE;
           st->first = 1;
           st->mode = MODE_HYBRID;
           st->bandwidth = OPUS_BANDWIDTH_FULLBAND;
           st->variable_HP_smth2_Q15 = silk_LSHIFT( silk_lin2log( VARIABLE_HP_MIN_CUTOFF_HZ ), 8 );
        }
        break;
        case OPUS_SET_FORCE_MODE_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            if ((value < MODE_SILK_ONLY || value > MODE_CELT_ONLY) && value != OPUS_AUTO)
            {
               goto bad_arg;
            }
            st->user_forced_mode = value;
        }
        break;
        case OPUS_SET_LFE_REQUEST:
        {
            opus_int32 value = va_arg(ap, opus_int32);
            st->lfe = value;
            ret = celt_encoder_ctl(celt_enc, OPUS_SET_LFE(value));
        }
        break;
        case OPUS_SET_ENERGY_MASK_REQUEST:
        {
            opus_val16 *value = va_arg(ap, opus_val16*);
            st->energy_masking = value;
            ret = celt_encoder_ctl(celt_enc, OPUS_SET_ENERGY_MASK(value));
        }
        break;

        case CELT_GET_MODE_REQUEST:
        {
           const CELTMode ** value = va_arg(ap, const CELTMode**);
           if (!value)
           {
              goto bad_arg;
           }
           ret = celt_encoder_ctl(celt_enc, CELT_GET_MODE(value));
        }
        break;
        default:
            /* fprintf(stderr, "unknown opus_encoder_ctl() request: %d", request);*/
            ret = OPUS_UNIMPLEMENTED;
            break;
    }
    va_end(ap);
    return ret;
bad_arg:
    va_end(ap);
    return OPUS_BAD_ARG;
}

void opus_encoder_destroy(OpusEncoder *st)
{
    opus_free(st);
}
