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
    int          celt_enc_offset;//celt状态的偏移地址
    int          silk_enc_offset;//silk状态的偏移地址
    silk_EncControlStruct silk_mode;//silk控制编码器
    int          application;//opus的应用模式
    int          channels;//声道数
    int          delay_compensation;//延迟补偿
    int          force_channels;//用户固定的声道数，如果输出为立体声则应该设置为非OPUS_AUTO，且输入音频为立体声
    int          signal_type;//语音信号类型，会根据此参数确定音频中语音概率（voice_est），三种选择OPUS_AUTO（48） OPUS_SIGNAL_VOICE（127） OPUS_SIGNAL_MUSIC（0）
                                                                             //后面会根据voice_est再次计算当前语音帧的带宽。
    int          user_bandwidth;//用户固定的带宽，如果不是OPUS_AUTO，则带宽会强制变为此值，OPUS_BANDWIDTH_NARROWBAND OPUS_BANDWIDTH_MEDIUMBAND OPUS_BANDWIDTH_WIDEBAND
    int          max_bandwidth;//最大带宽
    int          user_forced_mode;//用户指定的模式
    int          voice_ratio;//silk中默认为-1，当signal_type信号类型未指定时，可以指定语音概率
    opus_int32   Fs;//音频采样率
    int          use_vbr;//使用vbr模式，相对CBR模式而言
    int          vbr_constraint; //限制vbr的输出(实时使用更安全)
    int          variable_duration;//一帧处理的时间，应该和帧大小相对应，否则例如帧大小设置40ms，variable_duration设置为20ms，则会将40ms的数据当作20ms来处理，相当于信息有丢失且这些信息原来是在40ms时间内承载，现在放到20ms内，相当于加快一倍的播放速度
    opus_int32   bitrate_bps;//实际比特率，会根据声道数，模式，复杂度，丢包率来更新20ms的等效比特率，并通过此参数计算目标的输出字节数。此参数实际上与默认值无关，会根据user_bitrate_bps再次计算
    opus_int32   user_bitrate_bps;//可以设定为OPUS_AUTO（会根据帧率也就是和帧大小相关的来计算实际比特率也就是bitrate_bps = 60*st->Fs/frame_size + st->Fs*st->channels）
                                         // OPUS_BITRATE_MAX（实际比特率 = max_data_bytes*8*st->Fs/frame_size），和指定比特率值（返回指定的值）。此参数会计算转换成bitrate_bps值
    int          lsb_depth;//采样位数，silk中为16
    int          encoder_buffer;//编码当前帧时保留上一帧的数据个数
    int          lfe;//未知??
    int          arch;//当前架构
    int          use_dtx;//使用不连续传输，超过400ms的静音段会单独发一帧表示静音                 /* SILK 和 CELT 的通用 DTX  general DTX for both SILK and CELT */
#ifndef DISABLE_FLOAT_API
    TonalityAnalysisState analysis;
#endif

#define OPUS_ENCODER_RESET_START stream_channels
    int          stream_channels;//处理的声道数
    opus_int16   hybrid_stereo_width_Q14;//控制立体声效果的参数
    opus_int32   variable_HP_smth2_Q15;//高通滤波器的系数，用于最开始的滤波（去低频）中，会决定滤波器的截至频率
    opus_val16   prev_HB_gain;//celt中使用，上一帧的子带增益
    opus_val32   hp_mem[4];//？？高通滤波器状态
    int          mode;//当前的编码模式
    int          prev_mode;//记录上一帧的mode
    int          prev_channels;//上一帧的声道数
    int          prev_framesize;//上一帧的帧大小
    int          bandwidth;//当前带宽
    /* Bandwidth determined automatically from the rate (before any other adjustment) */
    int          auto_bandwidth;//根据音频计算的带宽，只有满足st->mode == MODE_CELT_ONLY || st->first || st->silk_mode.allowBandwidthSwitch才会自动选择带宽
    int          silk_bw_switch;//silk中转换带宽
    /* Sampling rate (at the API level) */
    int          first;//初始化编码器状态的标志
    opus_val16 * energy_masking;//掩蔽效应能量
    StereoWidthState width_mem;//？？
    opus_val16   delay_buffer[MAX_ENCODER_BUFFER*2];//延迟缓存的数据
#ifndef DISABLE_FLOAT_API
    int          detected_bandwidth;
    int          nb_no_activity_frames;
    opus_val32   peak_signal_energy;
#endif
    int          nonfinal_frame; /* current frame is not the final in a packet */
    opus_uint32  rangeFinal;//CELT编码器会将一些额外的校验信息添加到最后一个数据包中，以确保解码器能够正确地解析比特流。而当st->rangeFinal != 0时，CELT编码器不会添加这些冗余信息，从而可以减少输出比特率。
};

/* 音频信号处理中的状态转移表10000, 1000, ( NB<->MB)  11000, 1000,  (MB<->WB )13500, 1000,(  WB<->SWB)14000, 2000, ( SWB<->FB )第一列是中间阈值（无记忆），第二列是滞后值（与中间阈值之间的差异）Transition tables for the voice and music. First column is themiddle (memoriless) threshold. The second column is the hysteresis与中间阈值之间的差异(difference with the middle) */
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
    ret = silk_Get_Encoder_Size( &silkEncSizeBytes );//就是用sizeof得到silk_encoder结构体的大小位17384字节
    if (ret)
        return 0;
    silkEncSizeBytes = align(silkEncSizeBytes);
    celtEncSizeBytes = celt_encoder_get_size(channels);//celtEncSizeBytes=4996
    return align(sizeof(OpusEncoder))+silkEncSizeBytes+celtEncSizeBytes;
}

int opus_encoder_init(OpusEncoder* st, opus_int32 Fs, int channels, int application)
{   
    //指向 SILK 编码器状态的指针
    void *silk_enc;
    //指向 celt 编码器状态的指针
    CELTEncoder *celt_enc;
    int err;
    int ret, silkEncSizeBytes;

   if((Fs!=48000&&Fs!=24000&&Fs!=16000&&Fs!=12000&&Fs!=8000)||(channels!=1&&channels!=2)||
        (application != OPUS_APPLICATION_VOIP && application != OPUS_APPLICATION_AUDIO
        && application != OPUS_APPLICATION_RESTRICTED_LOWDELAY))
        return OPUS_BAD_ARG;

    OPUS_CLEAR((char*)st, opus_encoder_get_size(channels));//先将encoder指向的结构体初始化，将st指针所指位置后面的24588字节全部初始化opus_encoder_get_size(channels)=24588字节
    /* Create SILK encoder */
    ret = silk_Get_Encoder_Size( &silkEncSizeBytes );//SILK编码器结构体的实际大小silkEncSizeBytes=17384，还剩7204个字节，无差错则返回ret=0
    if (ret)
        return OPUS_BAD_ARG;
    silkEncSizeBytes = align(silkEncSizeBytes);//silkEncSizeBytes=17384
    st->silk_enc_offset = align(sizeof(OpusEncoder));//sizeof(OpusEncoder)=2208,st->silk_enc_offset=2208
    st->celt_enc_offset = st->silk_enc_offset+silkEncSizeBytes;//17384+2208=19592

    //encoder=st结构如下
    /*encoder一共24588个字节********************第2208个字节******************************************************************第19592个字节*********************************/
    /*      opus编码器状态占据2208个字节         silk_enc                中间17384个字节是silk编码器状态                      celt_enc   后面4996个字节是celt编码器状态  */
    /***********************************************************************************************************************************************************************/

    silk_enc = (char*)st+st->silk_enc_offset;
    celt_enc = (CELTEncoder*)((char*)st+st->celt_enc_offset);
    st->stream_channels = st->channels = channels;//流的声道数=opus声道数=音频声道数

    st->Fs = Fs;    //opus编码器采样率=音频采样率
    st->arch = opus_select_arch();//返回一个整数值，是架构类型，包括x86、ARM、MIPS等

    ret = silk_InitEncoder( silk_enc, st->arch, &st->silk_mode );//重置SILK 编码器，成功返回0
    if(ret)return OPUS_INTERNAL_ERROR;

    /* 预设slik控制器参数  default SILK parameters */
    st->silk_mode.nChannelsAPI              = channels;//传入opus的声道数和音频声道数一致
    st->silk_mode.nChannelsInternal         = channels;//opus内部处理数据的声道数和音频声道数一致
    st->silk_mode.API_sampleRate            = st->Fs;//传入opus的音频采样率
    st->silk_mode.maxInternalSampleRate     = 16000;//最大opus内部采样率，silkonly模式下会重新赋值为16000
    st->silk_mode.minInternalSampleRate     = 8000;//原8000，最小opus内部采样率，silkonly模式下会重新赋值为8000
    st->silk_mode.desiredInternalSampleRate = 8000;//原16000，Opus编解码器内部使用的采样率，只有8k，16k和12k可以选择，后面根据当前带宽重新赋值，如果是窄带则选择8k，不需要上下采样
    st->silk_mode.payloadSize_ms            = 20;//silk编码过程中会再次计算，使其与预设的framesize相匹配       silk_only模式下 =1000*framesize/fs
    st->silk_mode.bitRate                   = 6000;//原25k，压缩码流目标速率bps，后面根据这个值计算每一帧压缩后输出的位数
    st->silk_mode.packetLossPercentage      = 0;//原0
    st->silk_mode.complexity                = 3;//1-10，指在opus内部运算的复杂度，会决定基频估计器的复杂度，基频估计器的阙值，基频分析中白化滤波器的阶数，噪声整形滤波器的阶数
                                                               //基频分析学习样本个数，量化判决的状态数量，是否采用NLSF，NLSF_MSVQ剩余的个数，扭曲噪声整形的扭曲参数初值
    st->silk_mode.useInBandFEC              = 0;//不采用FEC，前向纠错，需要增加冗余数据用来验证包是否出错
    st->silk_mode.useDTX                    = 0;//0则不检测静音段，1则检测静音段，超过400ms则单独发一帧标志此静音段
    st->silk_mode.useCBR                    = 0;//原来0，使用恒定比特率模式
    st->silk_mode.reducedDependency         = 0;//帧与帧之间相关，设置为1则相互独立，会增加每一帧携带的信息增加码率

    /* 创造并初始化celt编码器Create CELT encoder */
    /* Initialize CELT encoder */
    err = celt_encoder_init(celt_enc, Fs, channels, st->arch);//初始化celt编码器并预设celt编码器的一部分参数，由于只考虑语音的压缩因此不详细注明celt

    if(err!=OPUS_OK)return OPUS_INTERNAL_ERROR;

    celt_encoder_ctl(celt_enc, CELT_SET_SIGNALLING(0));//用于关闭信令（signalling）选项，信令选项允许音频编解码器向解码器发送一些额外的信息，以帮助解码器更好地解码音频数据。这些信息可以包括通道配置、位深度、采样率等。
                                                       //例如：如果编码器和解码器之间的信道非常稳定，并且采用相同的配置，则不需要传输信令信息，因为解码器已经知道这些信息。
    celt_encoder_ctl(celt_enc, OPUS_SET_COMPLEXITY(st->silk_mode.complexity));//将st->silk_mode.complexity强制转换为opus_int32类型，如果x等于0，则不会发生任何事情

    /* 预设opus编码器参数*/
    st->use_vbr = 1;//使用vbr模式，相对CBR模式而言
    st->vbr_constraint = 1;//原1           /* 限制vbr的输出(实时使用更安全)Makes constrained VBR the default (safer for real-time use) */
    st->user_bitrate_bps = 6000;//可以设定为OPUS_AUTO（会根据帧率也就是和帧大小相关的来计算实际比特率也就是bitrate_bps = 60*st->Fs/frame_size + st->Fs*st->channels）
                                         // OPUS_BITRATE_MAX（实际比特率 = max_data_bytes*8*st->Fs/frame_size），和指定比特率值（返回指定的值）。此参数会计算转换成bitrate_bps值
    st->bitrate_bps =  Fs * channels;//实际比特率，会根据声道数，模式，复杂度，丢包率来更新20ms的等效比特率，并通过此参数计算目标的输出字节数。此参数实际上与默认值无关，会根据user_bitrate_bps再次计算
    st->application = application;//对应opus的应用模式
    st->signal_type = OPUS_SIGNAL_VOICE;//语音信号类型，会根据此参数确定音频中语音水平（voice_est），三种选择OPUS_AUTO（48） OPUS_SIGNAL_VOICE（127） OPUS_SIGNAL_MUSIC（0）
                                                                             //后面会根据voice_est再次计算当前语音帧的带宽。
    st->user_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//用户固定的带宽，如果不是OPUS_AUTO，则带宽会强制变为此值，OPUS_BANDWIDTH_NARROWBAND OPUS_BANDWIDTH_MEDIUMBAND OPUS_BANDWIDTH_WIDEBAND
    st->max_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//带宽上限，实际带宽会小于此参数
    st->force_channels = OPUS_AUTO;//用户固定的声道数，如果输出为立体声则应该设置为非OPUS_AUTO，且输入音频为立体声
    st->user_forced_mode = MODE_SILK_ONLY;//如果没有强制设定模式OPUS_AUTO，则会根据signal_type计算的voice_est并配合语音音乐的比特率基础值来计算比特率门限，来选择模式；
                                          //如果强制设定了模式则不会计算以上，直接强制设定为此模式 MODE_HYBRID  MODE_CELT_ONLY MODE_SILK_ONLY
    st->voice_ratio = 1;//在编码过程中会设置为-1，针对OPUS_APPLICATION_AUDIO模式，增加voice_est
    st->encoder_buffer = st->Fs / 100;//编码当前帧时，保留上一帧的音频数据的个数，过大会增加延迟
    st->lsb_depth =16;     //量化进度，也就是采样位数，主要用于celt，silk中默认16
    st->variable_duration = OPUS_FRAMESIZE_ARG;//默认设置参数用来选择帧大小的参数，原OPUS_FRAMESIZE_ARG
    st->delay_compensation = st->Fs/250;/* 延迟补偿4毫秒(SILK 的额外前瞻2.5毫秒 + SILK 重采样和立体预测1.5毫秒)Delay compensation of 4 ms (2.5 ms for SILK's extra look-ahead+1.5 ms for SILK resamplers and stereo prediction) */
    st->hybrid_stereo_width_Q14 = 1 << 14;//控制立体声效果的参数
    st->prev_HB_gain = Q15ONE;//上一帧的子带增益，主要用于celt判断中
    st->variable_HP_smth2_Q15 = silk_LSHIFT( silk_lin2log( VARIABLE_HP_MIN_CUTOFF_HZ ), 8 );//高通滤波器的系数，用于最开始的滤波（去低频）中，会决定滤波器的截至频率
    st->first = 1;//初始化编码器状态的标志，只有第一帧是1，或者发出重置指令时
    st->mode = MODE_SILK_ONLY;//当前的编码模式，默认情况下，CELT编码器和解码器的模式均为MODE_HYBRID。根据应用场景和需求，可以选择不同的模式以达到最佳的音频质量和数据传输效率
    st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;//当前带宽

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
   silk_assert( cutoff_Hz <= silk_int32_MAX / SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 ) );//检查cutoff_Hz是否正确 SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 )=2471检查cutoff_Hz是否小于(2147483647/2470)=869426
   
   Fc_Q19 = silk_DIV32_16( silk_SMULBB( SILK_FIX_CONST( 1.5 * 3.14159 / 1000, 19 ), cutoff_Hz ), Fs/1000 );//18532  =2471*cutoff_Hz/Fs/1000
   silk_assert( Fc_Q19 > 0 && Fc_Q19 < 32768 );//检查Fc_Q19在0到32768之间

   r_Q28 = SILK_FIX_CONST( 1.0, 28 ) - silk_MUL( SILK_FIX_CONST( 0.92, 9 ), Fc_Q19 );//259706884
   //手动选择AR和MA系数
   /* b = r * [ 1; -2; 1 ]; */
   /* a = [ 1; -2 * r * ( 1 - 0.5 * Fc^2 ); r^2 ]; */
   B_Q28[ 0 ] = r_Q28;
   B_Q28[ 1 ] = silk_LSHIFT( -r_Q28, 1 );//-519413768
   B_Q28[ 2 ] = r_Q28;
   /* -r * ( 2 - Fc * Fc ); */
   r_Q22  = silk_RSHIFT( r_Q28, 6 );// r_Q22=4055646
   A_Q28[0] = silk_SMULWW(r_Q22, silk_SMULWW(Fc_Q19, Fc_Q19) - SILK_FIX_CONST(2.0, 22));//((r_Q22* (Fc_Q19* Fc_Q19 >> 16) - (2.0 * 1 << 22 + 0.5) >> 16)       A_Q28[ 0 ]=-518787523
   A_Q28[ 1 ] = silk_SMULWW( r_Q22, r_Q22 );//r_Q22* r_Q22 >> 16     A_Q28[ 0 ]=-519089305，A_Q28[ 1 ]=251262126
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

//如果编码器初始化，返回分配内存的首地址，动态分配地址，分配24588个byte，并将首地址返回
OpusEncoder *opus_encoder_create(opus_int32 Fs, int channels, int application, int *error)
{
   int ret;
   OpusEncoder *st;//OpusEncoder类型的指针变量
   if((Fs!=48000&&Fs!=24000&&Fs!=16000&&Fs!=12000&&Fs!=8000)||(channels!=1&&channels!=2)||
       (application != OPUS_APPLICATION_VOIP && application != OPUS_APPLICATION_AUDIO
       && application != OPUS_APPLICATION_RESTRICTED_LOWDELAY))//采样率不是48000，24000，16000，12000，8000，或者，声道数上不为1或2，或者编码器模式不是那三种模式返回空指针也算报错
   {  
      if (error)
         *error = OPUS_BAD_ARG;
      return NULL;
   }
   st = (OpusEncoder *)opus_alloc(opus_encoder_get_size(channels));//动态分配地址，分配24588个byte，并将首地址赋值给st指针
   if (st == NULL)
   {
      if (error)
      {
          *error = OPUS_ALLOC_FAIL;
          printf("动态分配失败");//
      }

      return NULL;
   }
   ret = opus_encoder_init(st, Fs, channels, application);//这里同时初始化celt和silk的编码器状态,以及预设编码器的参数
                                                         //初始化编码器状态并返回是否成功OPUS_OK和errors
   //printf("*st->variable_duration= %d ", st->variable_duration);结果是5000
   if (error)
      *error = ret;
   if (ret != OPUS_OK)
   {
      opus_free(st);//编码失败释放内存
      st = NULL;
   }
   return st;
}
//将用户设定比特率转化为实际比特率OPUS_AUTO（60*st->Fs/frame_size + st->Fs*st->channels），OPUS_BITRATE_MAX（max_data_bytes*8*st->Fs/frame_size），其余则直接赋值
static opus_int32 user_bitrate_to_bitrate(OpusEncoder *st, int frame_size, int max_data_bytes)
{
  if(!frame_size)frame_size=st->Fs/400;//默认5ms一帧
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
    //variable_duration=5000    默认参数,这个参数只在这里出现，目的就是选择合理的帧长，最好是和设定的帧长相匹配也就是相等，否则会出现倍速播放且音质下降，具体原因在OpusEncoder结构体中
       //variable_duration不能大于设置的帧大小，因为
       int new_size;
       if (frame_size<Fs/400)  //帧大小比2.5ms小
       {
           printf("frame_size<Fs/400");
           return -1;
       }
       if (variable_duration == OPUS_FRAMESIZE_ARG)   //如果参数variable_duration是默认值则不选择帧长，直接采用设定的frame_size帧长，否则根据更改的默认值选择帧长
           new_size = frame_size;
       else if (variable_duration >= OPUS_FRAMESIZE_2_5_MS && variable_duration <= OPUS_FRAMESIZE_120_MS)//variable_duration也是控制在2.5ms到120ms之间
       {                                            
           if (variable_duration <= OPUS_FRAMESIZE_40_MS)                          //这里的计算是为了实现让新计算的帧大小和variable_duration设置的值相等
               new_size = (Fs / 400) << (variable_duration - OPUS_FRAMESIZE_2_5_MS);
           else
               new_size = (variable_duration - OPUS_FRAMESIZE_2_5_MS - 2) * Fs / 50;
       }
       else
           return -1;
       if (new_size>frame_size)                                                    //根据variable_duration计算的新帧长不会超过设定的帧长
          return -1;
       if (400*new_size!=Fs   && 200*new_size!=Fs   && 100*new_size!=Fs   &&
            50*new_size!=Fs   &&  25*new_size!=Fs   &&  50*new_size!=3*Fs &&
            50*new_size!=4*Fs &&  50*new_size!=5*Fs &&  50*new_size!=6*Fs)         //最后检验帧长是否合理
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
      /* 计算在当前带宽设置下使用FEC的阈值Compute threshold for using FEC at the current bandwidth setting */
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
          printf("计算LBRR函数返回1");
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

/* 返回对应于20ms 帧的等效比特率，每次基于bitrate进行计算，涉及帧率，通道数，vbr，模式，复杂度，丢包率Returns the equivalent bitrate corresponding to 20 ms frames,
   complexity 10 VBR operation. */
static opus_int32 compute_equiv_rate(opus_int32 bitrate, int channels,
      int frame_rate, int vbr, int mode, int complexity, int loss)
{
   opus_int32 equiv;
   equiv = bitrate;//9000
   /* 考虑较小帧的开销。Take into account overhead from smaller frames. */
   equiv -= (40*channels+20)*(frame_rate - 50);            //帧长大于20ms则增加比特率
   /* CBR is about a 8% penalty for both SILK and CELT. */
   if (!vbr)
      equiv -= equiv/12;                                   //采用cbr模式则减少8%的实际比特率
   /* 一般来说，复杂性会产生10% 的差异(从0到10)。Complexity makes about 10% difference (from 0 to 10) in general. */
   equiv = equiv * (90+complexity)/100;                    //根据复杂度再次计算实际比特率，取(90+complexity)/100倍的比特率
   if (mode == MODE_SILK_ONLY || mode == MODE_HYBRID)      //涉及到silk模式会根据loss？？减少实际比特率，如果复杂度小于2，则取0.8倍的实际比特率
                                                           //celt模式且复杂度小于5取0.9倍的实际比特率
                                                           //OPUS_AUTO则根据loss减少实际比特率
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

opus_int32 opus_encode_native(OpusEncoder *st,/*opus编码状态器*/
                              const opus_val16 *pcm, /*音频数据，一帧*/
                              int frame_size,/*和variable_duration匹配后的帧大小*/
                              unsigned char *data, /*输出数据*/
                              opus_int32 out_data_bytes, /*输出数据个数*/
                              int lsb_depth,/*采样精度，固定16*/
                              const void *analysis_pcm, /*音频数据，一帧和pcm一样*/
                              opus_int32 analysis_size,/*用户设置的帧大小*/ 
                              int c1,/*0*/ int c2,/*-2*/
                              int analysis_channels, /*声道数*/
                              downmix_func downmix, /*一个函数？？*/
                              int float_api/*0*/)
{
    void *silk_enc;//silk编码器状态，silk控制器
    CELTEncoder *celt_enc;//celt编码器状态，celt控制器
    int i;
    int ret=0;
    opus_int32 nBytes;//输出的码元数目
    ec_enc enc;//用于存储压缩数据编码器的状态信息
    int bytes_target;//用于存储编码器的目标输出字节数
    int prefill=0;//用于控制编码器初始状态的一个整型变量, 0，即编码器的初始状态为空闲状态
    int start_band = 0;//用于控制编码器开始编码的频带数的一个整型变量
    int redundancy = 0;//为 0 时，编码器不会进行冗余编码
    int redundancy_bytes = 0; /* Number of bytes to use for redundancy frame 冗余帧的字节数*/
    int celt_to_silk = 0;//是否将 CELT 模式转换为 SILK 模式进行编码,为 0，则不进行转换
    VARDECL(opus_val16, pcm_buf);//定义了一个 opus_val16 类型的变量 pcm_buf，用于存储音频 PCM 数据的缓冲区。
    int nb_compr_bytes;// 存储的就是经过压缩后的 Opus 数据的字节数。
    int to_celt = 0;//是否将输入信号转换为 CELT 模式,转换的话可以在保持较高音频质量的同时实现更高的压缩率。原0
    opus_uint32 redundant_rng = 0;//指示是否启用冗余编码
    int cutoff_Hz, hp_freq_smth1;//cutoff_Hz 和 hp_freq_smth1 参数用于控制高通滤波器的截止频率和平滑程度。默认情况下，cutoff_Hz 的值为 150 Hz，这意味着编码器会去除输入信号中低于 150 Hz 的部分。默认情况下，hp_freq_smth1 的值为 1，表示使用最简单的平滑技术。
    int voice_est; /* Probability of voice in Q7 *///Q7语音概率
    opus_int32 equiv_rate;//存储的是编码器输出比特率的平均值，可以用来衡量编码器的整体压缩效率。
    int delay_compensation;//用来控制编码器在进行编码时预测延迟的大小，从而使得解码器能够准确地进行延迟补偿。32
    int frame_rate;//编码器的帧率，也就是每秒钟处理的音频帧数=16
    opus_int32 max_rate; /* Max bitrate we're allowed to use可使用的最大比特率 *///用于控制编码器的最大比特率163328
    int curr_bandwidth;//表示当前的编码带宽= st->bandwidth=1101<4khzNB
    opus_val16 HB_gain;//用于控制高频增益。当编码器检测到输入信号中存在高频内容时，它可以通过增加HB_gain来提高输出信号的高频部分。
    opus_int32 max_data_bytes;//表示Opus编码器的最大输出数据字节数的整数。该参数用于限制编码器的输出数据大小，以确保输出的数据不会超出指定的最大字节数。1276
    int total_buffer;//表示编码器内部的所有缓冲区总共可以容纳多少采样点32
    opus_val16 stereo_width;//用于控制立体声信号的宽度,只对立体声信号有效，对于单声道信号则没有任何影响=0
    const CELTMode *celt_mode;///将CELT编码器对象celt_enc中的模式信息保存到变量celt_mode中
#ifndef DISABLE_FLOAT_API
    AnalysisInfo analysis_info;
    int analysis_read_pos_bak=-1;
    int analysis_read_subframe_bak=-1;
    int is_silence = 0;
#endif
    VARDECL(opus_val16, tmp_prefill);
    ALLOC_STACK;
    max_data_bytes = IMIN(1276, out_data_bytes);                         //确定最大输出字节数
    st->rangeFinal = 0;
    if (frame_size <= 0 || max_data_bytes <= 0)                          //检查帧大小和最大压缩字节
    {
       RESTORE_STACK;
       printf("//帧大小小于=零或者最大数据的比特数小于=零//1234//\n");
       return OPUS_BAD_ARG;
    }

    /* Cannot encode 100 ms in 1 byte */
    if (max_data_bytes==1 && st->Fs==(frame_size*10))                   //最大压缩字节限制太小了
    {
      RESTORE_STACK;
      printf("//最大字节数为1且采样率=frame_size*10时返回opus的buffer太小//1242//\n");
      return OPUS_BUFFER_TOO_SMALL;
    }
                                                                        //把silk和celt编码器定位到指定位置
    silk_enc = (char*)st+st->silk_enc_offset;                           
    celt_enc = (CELTEncoder*)((char*)st+st->celt_enc_offset);

    if (st->application == OPUS_APPLICATION_RESTRICTED_LOWDELAY)        //如果采用低延迟模式则延迟buffer长度为0，如果采用其他模式buffer长度默认为延迟补偿初始值st->Fs/250，也就是延迟4ms
    {
        printf("达到最低延迟最重要，放弃语音优化");
        delay_compensation = 0;
    }
    else
       delay_compensation = st->delay_compensation;
    
    lsb_depth = IMIN(lsb_depth, st->lsb_depth);                         //调用函数时lsb_depth=16，因此除非在初始化时将 st->lsb_depth设置的更小，否则采样位数就是16

    celt_encoder_ctl(celt_enc, CELT_GET_MODE(&celt_mode));              //将CELT编码器模式信息保存到变量celt_mode中

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
          printf("进入静音判断\n");
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
       stereo_width = compute_stereo_width(pcm, frame_size, st->Fs, &st->width_mem);            //计算立体声信号的宽度，单声道则为0
    else
       stereo_width = 0;
    total_buffer = delay_compensation;                                                          //延迟buffer长度 ==st->Fs/250
    st->bitrate_bps = user_bitrate_to_bitrate(st, frame_size, max_data_bytes);                  //根据用户设定的比特率计算实际比特率，除非 user_bitrate为OPUS_AUTO或者OPUS_BITRATE_MAX，否则st->bitrate_bps=user_bitrate
    frame_rate = st->Fs / frame_size;                                                           //帧率，取决于设置的帧大小和当前音频的采样率，1s中有几帧
    if (!st->use_vbr)                                                                           //采用CBR，更新实际比特率，更新最大输出字节
    {
       int cbrBytes;
       /* Multiply by 12 to make sure the division is exact. */
       int frame_rate12 = 12*st->Fs/frame_size;
       /* We need to make sure that "int" values always fit in 16 bits. */
       cbrBytes = IMIN( (12*st->bitrate_bps/8 + frame_rate12/2)/frame_rate12, max_data_bytes);//计算cbr情况的输出字节
       st->bitrate_bps = cbrBytes*(opus_int32)frame_rate12*8/12;//更新cbr情况下的比特率
       /* Make sure we provide at least one byte to avoid failing. */
       max_data_bytes = IMAX(1, cbrBytes);
       printf("使用cbr模式");
    }
    if (max_data_bytes<3 || st->bitrate_bps < 3*frame_rate*8                                   //最大字节小于3或者一帧至少输出三个字节，或者帧率小于50（也就是一帧小于20ms）时全部满输出还小于300字节也就是比特率小于2400空间太小发出“ PLC”帧
       || (frame_rate<50 && (max_data_bytes*frame_rate<300 || st->bitrate_bps < 2400)))

    {
       /*如果空间太小，无法做有用的事情，发出“ PLC”帧 If the space is too low to do something useful, emit 'PLC' frames.*/
       int tocmode = st->mode;//编码模式（TOC mode）MODE_HYBRID
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

       data[0] = gen_toc(tocmode, frame_rate, bw, st->stream_channels);                                     //PLC帧中生成 Opus 封包中的 TOC 字段
       data[0] |= packet_code;                                                                              //PLC帧中当前封包中每个音频帧的长度信息,packet_code 赋值给 data[0] 的低位
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
    }                                                                                                         //PLC帧结束


    max_rate = frame_rate*max_data_bytes*8;                                                                  //计算得到最大编码比特率

    
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->channels, st->Fs/frame_size,st->use_vbr, 0, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);/* 根据模式信道数带宽再次计算实际比特率Equivalent 20-ms rate for mode/channel/bandwidth decisions */
    if (st->signal_type == OPUS_SIGNAL_VOICE)                                                               //st->signal_type是默认值且不会变化，如果模式采用OPUS_APPLICATION_VOIP压缩结果基本没差别
        voice_est = 127;//信号设定为语音则对应语音概率为1
    else if (st->signal_type == OPUS_SIGNAL_MUSIC)
       voice_est = 0;//信号设定为音乐则对应语音概率为0
    else if (st->voice_ratio >= 0)
    {
       voice_est = st->voice_ratio*327>>8;
       /* 对于一个音频只有90%的可能是语音信号For AUDIO, never be more than 90% confident of having speech */
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
          st->stream_channels = st->channels;                                                          //更新内部流的声道数
       }
#endif
    }
    /* 根据声道的选择更新平均比特率Update equivalent rate for channels decision. */
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->stream_channels, st->Fs/frame_size,st->use_vbr, 0, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);//输入为单声道值不变
    /* 根据应用程序和信号类型选择模式Mode selection depending on application and signal type */
    if (st->application == OPUS_APPLICATION_RESTRICTED_LOWDELAY)                                       //低延迟应用模式必须采用celt模式
    {                                                                                                 //如果没有强制设定模式则会计算比特率门限判定选择模式，低于门限则采用silk，高于门限则采用celt模式
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
        opus_int32 mode_voice;//单声道也就是语音概率值
        opus_int32 mode_music;//双声道也就是音乐概率值，通过概率的值来计算门限
       opus_int32 threshold;                                                                           //比特率门限

       /* 基于立体宽度的插值Interpolate based on stereo width */
       mode_voice = (opus_int32)(MULT16_32_Q15(Q15ONE-stereo_width,mode_thresholds[0][0])+ MULT16_32_Q15(stereo_width,mode_thresholds[1][0]));//单声道
       mode_music = (opus_int32)(MULT16_32_Q15(Q15ONE-stereo_width,mode_thresholds[1][1])+ MULT16_32_Q15(stereo_width,mode_thresholds[1][1]));//双声道
       /* 基于语音/音乐概率的插值Interpolate based on speech/music probability */
       threshold = mode_music + ((voice_est*voice_est*(mode_voice-mode_music))>>14);
       /* 由于一些有用的特性，对 VoIP 的 SILK 的偏好  Bias towards SILK for VoIP because of some useful features */
       if (st->application == OPUS_APPLICATION_VOIP)                                                    //采用voip模式则门限加8000
          threshold += 8000;
       //printf("%f %d\n", stereo_width/(float)Q15ONE, threshold);
       /* Hysteresis */
       if (st->prev_mode == MODE_CELT_ONLY)                                                             //采用celt则门限减4000（音乐占比较高，所以减去一些语音可能性）
           threshold -= 4000;
       else if (st->prev_mode>0)                                                                        //不是第一帧则门限加4000
           threshold += 4000;

       st->mode = (equiv_rate >= threshold) ? MODE_CELT_ONLY: MODE_SILK_ONLY;                           //等效比特率大于门限则celt否则silk（语音的比特率一般较小）
       /* When FEC is enabled and there's enough packet loss, use SILK */
       if (st->silk_mode.useInBandFEC && st->silk_mode.packetLossPercentage > (128-voice_est)>>4)       //如果设置丢包百分比并采用前向纠错（FEC）则silk
          st->mode = MODE_SILK_ONLY; 
       /* When encoding voice and DTX is enabled but the generalized DTX cannot be used,
          because of complexity and sampling frequency settings, switch to SILK DTX and
          set the encoder to SILK mode */
#ifndef DISABLE_FLOAT_API
       st->silk_mode.useDTX = st->use_dtx && !(analysis_info.valid || is_silence);
#else
       st->silk_mode.useDTX = st->use_dtx;                                        //use_dtx默认值传递给 st->silk_mode,一般不采用，同时根据dtx，语音基础值voice_est，以及max_data_bytes参数判断本帧采用的模式
#endif
       if (st->silk_mode.useDTX && voice_est > 100)
          st->mode = MODE_SILK_ONLY;
#endif

       /* If max_data_bytes represents less than 6 kb/s, switch to CELT-only mode */                 //这里的逻辑是framesize=fs*t(这里的t是一帧的时间)   max_bytes*8/t<6000(这个就是代表小于6kb/s),当帧率大于50时则不等式右边变成9000
       if (max_data_bytes < (frame_rate > 50 ? 9000 : 6000)*frame_size / (st->Fs * 8))
          st->mode = MODE_CELT_ONLY;
    } else {
       st->mode = st->user_forced_mode;                                                             //用户强制设定的模式，默认auto
    }

    /* 覆盖选定的模式，以确保我们满足要求的帧大小Override the chosen mode to make sure we meet the requested frame size */
    if (st->mode != MODE_CELT_ONLY && frame_size < st->Fs/100)                                       //帧大小小于10ms则celt
        st->mode = MODE_CELT_ONLY;
    if (st->lfe)
       st->mode = MODE_CELT_ONLY;

    if (st->prev_mode > 0 &&((st->mode != MODE_CELT_ONLY && st->prev_mode == MODE_CELT_ONLY) ||(st->mode == MODE_CELT_ONLY && st->prev_mode != MODE_CELT_ONLY)))
    {                                                                                                //当前帧不是第一帧，且上一帧或当前帧的模式是celt才会由celt转为silk，并增加1个冗余字节
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
    /* 当编码多帧时，我们只能要求在最后一帧切换到 CELT。这个开关处理上面的请求模式不应该中断立体声-> 单声道转换。When encoding multiframes, we can ask for a switch to CELT only in the last frame. This switch
    * is processed above as the requested mode shouldn't interrupt stereo->mono transition. */
    if (st->stream_channels == 1 && st->prev_channels ==2 && st->silk_mode.toMono==0                                          
          && st->mode != MODE_CELT_ONLY && st->prev_mode != MODE_CELT_ONLY)
    {
       /* 延迟两帧的立体声-> 单声道转换，这样 SILK 可以做一个平滑的混音Delay stereo->mono transition by two frames so that SILK can do a smooth downmix */
       st->silk_mode.toMono = 1;
       st->stream_channels = 2;
    } else {
       st->silk_mode.toMono = 0;
    }

    /* 根据模式决策更新等效速率，由于loss=0且复杂度>2所以没变化还是10929，如果复杂度<2则返回8743   Update equivalent rate with mode decision. */
    equiv_rate = compute_equiv_rate(st->bitrate_bps, st->stream_channels, st->Fs/frame_size,st->use_vbr, st->mode, st->silk_mode.complexity, st->silk_mode.packetLossPercentage);//根据复杂度，帧率更新内部采样率
    
    if (st->mode != MODE_CELT_ONLY && st->prev_mode == MODE_CELT_ONLY)
    {
        silk_EncControlStruct dummy;
        silk_InitEncoder( silk_enc, st->arch, &dummy);
        prefill=1;
    }

    /* 自动(依赖速率的)带宽选择Automatic (rate-dependent) bandwidth selection */
    if (st->mode == MODE_CELT_ONLY || st->first || st->silk_mode.allowBandwidthSwitch)              //根据语音检测voice_est的结果（opus_auto)重新计算各个带宽的门限
    {
        const opus_int32 *voice_bandwidth_thresholds, *music_bandwidth_thresholds;//将各个带宽的选择门限传递给这两个变量voice_bandwidth_thresholds，music_bandwidth_thresholds
        opus_int32 bandwidth_thresholds[8];
        int bandwidth = OPUS_BANDWIDTH_FULLBAND;//默认1105<20 kHz全频带
        if (st->channels==2 && st->force_channels!=1)
        {
           voice_bandwidth_thresholds = stereo_voice_bandwidth_thresholds;
           music_bandwidth_thresholds = stereo_music_bandwidth_thresholds;
        } else {//选择单声道门限
           voice_bandwidth_thresholds = mono_voice_bandwidth_thresholds;
           music_bandwidth_thresholds = mono_music_bandwidth_thresholds;
        }
        /*根据语音检测voice_est的结果（opus_auto)重新计算各个带宽的门限   Interpolate bandwidth thresholds depending on voice estimation */
        for (i=0;i<8;i++)//更新门限为9807，1000，10807，1000，13017，1000，13614，2000，门限只和语音检测voice_est的结果有关，也就是只和预设的信号类型和应用模式决定
        {
           bandwidth_thresholds[i] = music_bandwidth_thresholds[i] + ((voice_est*voice_est*(voice_bandwidth_thresholds[i]-music_bandwidth_thresholds[i]))>>14);
        }
        do {
            int threshold, hysteresis;
            threshold = bandwidth_thresholds[2*(bandwidth-OPUS_BANDWIDTH_MEDIUMBAND)];//门限=13614
            hysteresis = bandwidth_thresholds[2*(bandwidth-OPUS_BANDWIDTH_MEDIUMBAND)+1];//滞后=2000
            if (!st->first)//0
            {
                if (st->auto_bandwidth >= bandwidth)
                    threshold -= hysteresis;//门限减去滞后
                else
                    threshold += hysteresis;
            }
            if (equiv_rate >= threshold)
                break;
        } while (--bandwidth>OPUS_BANDWIDTH_NARROWBAND);
        st->bandwidth = st->auto_bandwidth = bandwidth;//=1103<8khz
        /* 防止任何过渡到 SWB/FB，直到 SILK 层完全切换到 WB 模式，并关闭可变 LP 滤波器Prevents any transition to SWB/FB until the SILK layer has fully
           switched to WB mode and turned the variable LP filter off */
        if (!st->first && st->mode != MODE_CELT_ONLY && !st->silk_mode.inWBmodeWithoutVariableLP && st->bandwidth > OPUS_BANDWIDTH_WIDEBAND)
            st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    }

    if (st->bandwidth>st->max_bandwidth)
       st->bandwidth = st->max_bandwidth;

    if (st->user_bandwidth != OPUS_AUTO)
        st->bandwidth = st->user_bandwidth;

    /* 这就阻止了我们在不安全的 CBR/max 速率下使用混合模式This prevents us from using hybrid at unsafe CBR/max rates */
    if (st->mode != MODE_CELT_ONLY && max_rate < 15000)
    {
       st->bandwidth = IMIN(st->bandwidth, OPUS_BANDWIDTH_WIDEBAND);
    }

    /* 防止 Opus 在高于 Nyquist 频率的输入信号上浪费位Prevents Opus from wasting bits on frequencies that are above
       the Nyquist rate of the input signal */
    if (st->Fs <= 24000 && st->bandwidth > OPUS_BANDWIDTH_SUPERWIDEBAND)
        st->bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND;
    if (st->Fs <= 16000 && st->bandwidth > OPUS_BANDWIDTH_WIDEBAND)
        st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    if (st->Fs <= 12000 && st->bandwidth > OPUS_BANDWIDTH_MEDIUMBAND)
        st->bandwidth = OPUS_BANDWIDTH_MEDIUMBAND;
    if (st->Fs <= 8000 && st->bandwidth > OPUS_BANDWIDTH_NARROWBAND)                                                      //为了符合奈奎斯特采样定律
        st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;//走这个让st->bandwidth=1101<4khzNB
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
    st->silk_mode.LBRR_coded = decide_fec(st->silk_mode.useInBandFEC, st->silk_mode.packetLossPercentage,st->silk_mode.LBRR_coded, st->mode, &st->bandwidth, equiv_rate);//计算是否编码LBRR帧 //不编码lbrr帧
    celt_encoder_ctl(celt_enc, OPUS_SET_LSB_DEPTH(lsb_depth));

    /* CELT 模式不支持中频，改用宽频CELT mode doesn't support mediumband, use wideband instead */
    if (st->mode == MODE_CELT_ONLY && st->bandwidth == OPUS_BANDWIDTH_MEDIUMBAND)
        st->bandwidth = OPUS_BANDWIDTH_WIDEBAND;
    if (st->lfe)
       st->bandwidth = OPUS_BANDWIDTH_NARROWBAND;

    curr_bandwidth = st->bandwidth;

    /* Chooses the appropriate mode for speech选择合适的语音模式 永远不要在这里切换到或切换到 CELT 模式，因为这会使一些假设失效
       *NEVER* switch to/from CELT-only mode here as this will invalidate some assumptions */
    if (st->mode == MODE_SILK_ONLY && curr_bandwidth > OPUS_BANDWIDTH_WIDEBAND)//>8khz就是混合模式
        st->mode = MODE_HYBRID;
    if (st->mode == MODE_HYBRID && curr_bandwidth <= OPUS_BANDWIDTH_WIDEBAND)//<8khz就是只用silk模式
        st->mode = MODE_SILK_ONLY;

    /* 不进入，无法处理高于60毫秒的音频帧，在混合或CELT-only模式下，不能处理高于20毫秒的音频帧,为了能处理进行分帧操作Can't support higher than >60 ms frames, and >20 ms when in Hybrid or CELT-only modes */
    if ((frame_size > st->Fs/50 && (st->mode != MODE_SILK_ONLY)) || frame_size > 3*st->Fs/50)
    {
       int enc_frame_size;//编码一帧的帧大小
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
    /* 一个新的 SILK 带宽的第一帧For the first frame at a new SILK bandwidth */
    if (st->silk_bw_switch)//st->silk_bw_switch=0
    {
       redundancy = 1;
       celt_to_silk = 1;
       st->silk_bw_switch = 0;
       prefill=1;
    }

    /*如果我们决定使用 CELT，无论我们之前做了什么决定，都要确保关闭冗余If we decided to go with CELT, make sure redundancy is off, no matter what
       we decided earlier. */
    if (st->mode == MODE_CELT_ONLY)
        redundancy = 0;
    if (redundancy)// redundancy = 0如果开启冗余就计算冗余字节
    {
       redundancy_bytes = compute_redundancy_bytes(max_data_bytes, st->bitrate_bps, frame_rate, st->stream_channels);
       if (redundancy_bytes == 0)
          redundancy = 0;
    }
     /*printf("%d %d %d %d\n", st->bitrate_bps, st->stream_channels, st->mode, curr_bandwidth);*/
    bytes_target = IMIN(max_data_bytes-redundancy_bytes, st->bitrate_bps * frame_size / (st->Fs * 8)) - 1;//计算目标字节
                                                                                                   //这里跳过一个字节是为TOC字段预留的，是因为在编码完之后再填充TOC字段 
    data += 1;
                                                                                                   //区间编码器初始化，每一帧都会初始化，这样当存在网络丢包、抖动时帧间是不会有影响的
    ec_enc_init(&enc, data, max_data_bytes-1);

    ALLOC(pcm_buf, (total_buffer+frame_size)*st->channels, opus_val16);
    OPUS_COPY(pcm_buf, &st->delay_buffer[(st->encoder_buffer-total_buffer)*st->channels], total_buffer*st->channels);    //delay_buffer中存的是上一帧的4ms的数据，第一帧全零
                                                                                                                         //根据是语音还是music情况确定高通平滑频率
    if (st->mode == MODE_CELT_ONLY)
       hp_freq_smth1 = silk_LSHIFT( silk_lin2log( VARIABLE_HP_MIN_CUTOFF_HZ ), 8 );
    else
       hp_freq_smth1 = ((silk_encoder*)silk_enc)->state_Fxx[0].sCmn.variable_HP_smth1_Q15;                             //高通平滑参数，和上一帧的语音活动水平有关

    st->variable_HP_smth2_Q15 = silk_SMLAWB( st->variable_HP_smth2_Q15,hp_freq_smth1 - st->variable_HP_smth2_Q15, SILK_FIX_CONST( VARIABLE_HP_SMTH_COEF2, 16 ) ); //根据上一帧的语音活动水平机上一帧的平滑参数进行更新参数
    /* convert from log scale to Hertz */
    cutoff_Hz = silk_log2lin( silk_RSHIFT( st->variable_HP_smth2_Q15, 8 ) );                            //将平滑参数计算为频率
                                                                                                        //根据使用模式，进行预滤波，music场景只去除直流；对于语音，低于截止频率cutoff_Hz以下的信号因人类发声不会低于该频率
    if (st->application == OPUS_APPLICATION_VOIP)
    {
       hp_cutoff(pcm, cutoff_Hz, &pcm_buf[total_buffer*st->channels], st->hp_mem, frame_size, st->channels, st->Fs, st->arch);//去低频
    } else {
       dc_reject(pcm, 3, &pcm_buf[total_buffer*st->channels], st->hp_mem, frame_size, st->channels, st->Fs);//隔直流
    }
                                                                                                         //OPUS_APPLICATION_VOIP和OPUS_APPLICATION_AUDIO模式的区别就在于上边两个函数的选择
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
    if (st->mode != MODE_CELT_ONLY)  //模式只要不是celt_only就进入，这里是先计算silk情况
    {
        opus_int32 total_bitRate, celt_rate;
        opus_int activity;
#ifdef FIXED_POINT
       const opus_int16 *pcm_silk;
#else
       VARDECL(opus_int16, pcm_silk);
       ALLOC(pcm_silk, st->channels*frame_size, opus_int16);
#endif

        activity = VAD_NO_DECISION;                                                                              //重置活动检测参数
#ifndef DISABLE_FLOAT_API
        if( analysis_info.valid ) {
            /* Inform SILK about the Opus VAD decision */
            activity = ( analysis_info.activity_probability >= DTX_ACTIVITY_THRESHOLD );
        }
#endif

        /* 将位分配给SILK和CELTDistribute bits between SILK and CELT */
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
            st->silk_mode.bitRate = total_bitRate;                                                                //将所有位给silk编码器
        }
        /* 掩蔽效应silkSurround masking for SILK   掩蔽效应没有计算，默认=0则不进入*/
        if (st->energy_masking && st->use_vbr && !st->lfe)                                                         //energy_masking=0默认值
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

        st->silk_mode.payloadSize_ms = 1000 * frame_size / st->Fs;                                           //将framesize转换为ms，将声道数传给silk_mode结构体
        st->silk_mode.nChannelsAPI = st->channels;
        st->silk_mode.nChannelsInternal = st->stream_channels;
        if (curr_bandwidth == OPUS_BANDWIDTH_NARROWBAND)                                                     //当前帧的带宽决定了期望采样率，窄带对应8000，中带宽对应12000，宽带对应16000
            st->silk_mode.desiredInternalSampleRate = 8000;//走这个
        else 
            if (curr_bandwidth == OPUS_BANDWIDTH_MEDIUMBAND)
            st->silk_mode.desiredInternalSampleRate = 12000;
            else 
            {
            celt_assert( st->mode == MODE_HYBRID || curr_bandwidth == OPUS_BANDWIDTH_WIDEBAND );
            st->silk_mode.desiredInternalSampleRate = 16000;
            }
        if( st->mode == MODE_HYBRID ) {
            /* 在混合模式下，不要在最低比特率时允许带宽降低。Don't allow bandwidth reduction at lowest bitrates in hybrid mode 混合模式中由于celt的原因，认为当前音频中有音乐，因此将最小的采样率设置为16000*/
            st->silk_mode.minInternalSampleRate = 16000;
        } else {
            st->silk_mode.minInternalSampleRate = 8000;                                                       //根据编码模式设定最小内部采样率和最大内部采样率
        }                                                                                                     //只使用silk模式时，最小内部采样率就设置为8000

        st->silk_mode.maxInternalSampleRate = 16000;                                                         //只要不是celt模式最大内部采样率就设置为16000
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

        st->silk_mode.useCBR = !st->use_vbr;                                                                 //cbr和vbr不能同时开启

        /* Call SILK encoder for the low band */

        /* 对于SILK编码的音频数据，考虑到ToC（Type of Content）帧头信息、冗余字节和可选的冗余码字节，所能达到的最大比特数Max bits for SILK, counting ToC, redundancy bytes, and optionally redundancy. */
        st->silk_mode.maxBits = (max_data_bytes-1)*8;//=10200                                                //silk帧的最大字节数
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
            /* 将SILK前填充的开始部分设置为平滑过渡，避免编码器尝试编码间断。确切的位置是我们需要避免与冗余CELT帧混合时在音频中留下任何“空隙”。在这里，我们可以覆盖st->delay_buffer，因为在它被重写之前唯一使用它的是tmp_prefill[]，即使是使用的部分也只有过渡区域，而非发送到编码器并丢弃的部分。Use a smooth onset for the SILK prefill to avoid the encoder trying to encode
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
            //silk编码
            silk_Encode( silk_enc, &st->silk_mode, pcm_silk, st->encoder_buffer, NULL, &zero, 1, activity );
        }

#ifdef FIXED_POINT
        pcm_silk = pcm_buf+total_buffer*st->channels;//地址更新往后32位
#else
        for (i=0;i<frame_size*st->channels;i++)
            pcm_silk[i] = FLOAT2INT16(pcm_buf[total_buffer*st->channels + i]);
#endif
//short类型pcm数据存储首地址pcm_silk
 //frame_size:480 8kHz 60ms帧长
 //nByte:编码的字节数
 //activity：指示语音存在与否
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
               curr_bandwidth = OPUS_BANDWIDTH_NARROWBAND;//走这个
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
    if (st->mode != MODE_SILK_ONLY)//不走
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
    if (st->mode != MODE_SILK_ONLY && st->mode != st->prev_mode && st->prev_mode > 0)//不走
    {
        printf("opus_encoder.c 2068\n");
       OPUS_COPY(tmp_prefill, &st->delay_buffer[(st->encoder_buffer-total_buffer-st->Fs/400)*st->channels], st->channels*st->Fs/400);
    }

    if (st->channels*(st->encoder_buffer-(frame_size+total_buffer)) > 0)//不走
    {
        printf("opus_encoder.c 2074\n");
       OPUS_MOVE(st->delay_buffer, &st->delay_buffer[st->channels*frame_size], st->channels*(st->encoder_buffer-frame_size-total_buffer));
       OPUS_COPY(&st->delay_buffer[st->channels*(st->encoder_buffer-frame_size-total_buffer)],
             &pcm_buf[0],
             (frame_size+total_buffer)*st->channels);
    } else {
       OPUS_COPY(st->delay_buffer, &pcm_buf[(frame_size+total_buffer-st->encoder_buffer)*st->channels], st->encoder_buffer*st->channels);//将上一帧的前total_buffer个元素加到下一帧前面
    }
    /* gain_fade() and stereo_fade() need to be after the buffer copying
       because we don't want any of this to affect the SILK part */
    if( st->prev_HB_gain < Q15ONE || HB_gain < Q15ONE ) {//不走
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
    if( !st->energy_masking && st->channels == 2 ) {//不走
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
        if (st->mode == MODE_HYBRID)//不走
        {
            printf("opus_encoder.c 2124\n");
            ec_enc_bit_logp(&enc, redundancy, 12);
        }
        if (redundancy)//不走
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
        redundancy = 0;//只有这个有用
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
    } else {//不走
       nb_compr_bytes = (max_data_bytes-1)-redundancy_bytes;
       ec_enc_shrink(&enc, nb_compr_bytes);
    }

#ifndef DISABLE_FLOAT_API
    if (redundancy || st->mode != MODE_SILK_ONLY)
       celt_encoder_ctl(celt_enc, CELT_SET_ANALYSIS(&analysis_info));
#endif
    if (st->mode == MODE_HYBRID) {//不走
        printf("opus_encoder.c 2173\n");///////////////////////////////////////////////////////////
       SILKInfo info;
       info.signalType = st->silk_mode.signalType;
       info.offset = st->silk_mode.offset;
       celt_encoder_ctl(celt_enc, CELT_SET_SILK_INFO(&info));
    } else {
       celt_encoder_ctl(celt_enc, CELT_SET_SILK_INFO((SILKInfo*)NULL));//
    }

    /* 5 ms redundant frame for CELT->SILK */
    if (redundancy && celt_to_silk)//不走
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

    if (st->mode != MODE_SILK_ONLY)//不走
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
    if (redundancy && !celt_to_silk)//不走
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
    //根据silk_Encode结果，生成TOC字段
    data--;//
    data[0] = gen_toc(st->mode, st->Fs/frame_size, curr_bandwidth, st->stream_channels);                  //data中存储的是 TOC 字段和长度信息的组合值

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
    if (ec_tell(&enc) > (max_data_bytes-1)*8)//不走
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
    if (!st->use_vbr)//不走
    {
       if (opus_packet_pad(data, ret, max_data_bytes) != OPUS_OK)
       {
          RESTORE_STACK;
          return OPUS_INTERNAL_ERROR;
       }
       ret = max_data_bytes;//如果不采用vbr，每帧都是满字节压缩
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
//返回opus_encode_native调用结果，应该是编码的码元数量
opus_int32 opus_encode(OpusEncoder *st, const opus_int16 *pcm, int analysis_frame_size,
                unsigned char *data, opus_int32 out_data_bytes)
{
   int frame_size;
   frame_size = frame_size_select(analysis_frame_size, st->variable_duration, st->Fs);//analysis_frame_size：用户设置的帧大小，variable_duration：承载当前帧数据的时间
                                                                                      //目的是判断设置的帧大小是否合理（在2.5ms到120ms之间）
                                                                                      //如果variable_duration不是默认参数，则需要使variable_duration和analysis_frame_size匹配
   return opus_encode_native(st, pcm, frame_size, data, out_data_bytes, 16,pcm, analysis_frame_size, 0, -2, st->channels, downmix_int, 0);//编码函数
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
   //in:浮点格式的输入采用数据，frame_size:以采样点计数的帧长，data:编码后的字节序存放的地址，
//max_data_bytes:编码字节序的最大长度，lsb_depth:least significant bit 最低有效位数16
//pcm:short型音频数据，analysis_frame_size:分析帧长20ms，320点
//C1:0 C2:-2, 编码通道数st->channels, downmix_int是下采样函数地址所在，
//最后一个参数指示浮点API
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
