/***********************************************************************
Copyright (c) 2006-2011, Skype Limited. All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
- Neither the name of Internet Society, IETF or IETF Trust, nor the
names of specific contributors, may be used to endorse or promote
products derived from this software without specific prior written
permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef SILK_STRUCTS_H
#define SILK_STRUCTS_H

#include "typedef.h"
#include "SigProc_FIX.h"
#include "define.h"
#include "entenc.h"
#include "entdec.h"

#ifdef __cplusplus
extern "C"
{
#endif

/************************************/
/* Noise shaping quantization state */
/************************************/
typedef struct {
    opus_int16                  xq[           2 * MAX_FRAME_LENGTH ]; /* Buffer for quantized output signal                             */
    opus_int32                  sLTP_shp_Q14[ 2 * MAX_FRAME_LENGTH ];
    opus_int32                  sLPC_Q14[ MAX_SUB_FRAME_LENGTH + NSQ_LPC_BUF_LENGTH ];
    opus_int32                  sAR2_Q14[ MAX_SHAPE_LPC_ORDER ];
    opus_int32                  sLF_AR_shp_Q14;
    opus_int32                  sDiff_shp_Q14;
    opus_int                    lagPrev;
    opus_int                    sLTP_buf_idx;
    opus_int                    sLTP_shp_buf_idx;
    opus_int32                  rand_seed;
    opus_int32                  prev_gain_Q16;
    opus_int                    rewhite_flag;
} silk_nsq_state;

/********************************/
/* VAD state                    */
/********************************/
typedef struct {
    opus_int32                  AnaState[ 2 ];                  /* 分析滤波器组状态Analysis filterbank state: 0-8 kHz                                   */
    opus_int32                  AnaState1[ 2 ];                 /* Analysis filterbank state: 0-4 kHz                                   */
    opus_int32                  AnaState2[ 2 ];                 /* Analysis filterbank state: 0-2 kHz                                   */
    opus_int32                  XnrgSubfr[ VAD_N_BANDS ];       /* 子帧能量Subframe energies                                                    */
    opus_int32                  NrgRatioSmth_Q8[ VAD_N_BANDS ]; /* 每个频带中的能量噪声比25600->20dbSNR，后面用于计算每个频带的信噪比Smoothed energy level in each band                                   */
    opus_int16                  HPstate;                        /* 最低频带中的微分器状态State of differentiator in the lowest band                           */
    opus_int32                  NL[ VAD_N_BANDS ];              /* 每个子带中的噪声能量级Noise energy level in each band                                      */
    opus_int32                  inv_NL[ VAD_N_BANDS ];          /* 每个子带中的噪声反转能量级Inverse noise energy level in each band                              */
    opus_int32                  NoiseLevelBias[ VAD_N_BANDS ];  /* 噪声级估计器偏差/偏移Noise level estimator bias/offset                                    */
    opus_int32                  counter;                        /* 用来统计VAD的帧数，20ms为一帧，在开始设置为15，后面需要根据帧数来计算噪声平滑系数，前20s也就是前1000帧需要很快的平滑噪声Frame counter used in the initial phase                              */
} silk_VAD_state;

/* Variable cut-off low-pass filter state */
typedef struct {
    opus_int32                   In_LP_State[ 2 ];           /* 低通滤波器状态Low pass filter state */
    opus_int32                   transition_frame_no;        /* 映射到截止频率的计数器Counter which is mapped to a cut-off frequency */
    opus_int                     mode;                       /* 操作模式，>0上调，<0下调，=0不动Operating mode, <0: switch down, >0: switch up; 0: do nothing           */
} silk_LP_state;

/* Structure containing NLSF codebook */
typedef struct {
    const opus_int16             nVectors;
    const opus_int16             order;
    const opus_int16             quantStepSize_Q16;
    const opus_int16             invQuantStepSize_Q6;
    const opus_uint8             *CB1_NLSF_Q8;
    const opus_int16             *CB1_Wght_Q9;
    const opus_uint8             *CB1_iCDF;
    const opus_uint8             *pred_Q8;
    const opus_uint8             *ec_sel;
    const opus_uint8             *ec_iCDF;
    const opus_uint8             *ec_Rates_Q5;
    const opus_int16             *deltaMin_Q15;
} silk_NLSF_CB_struct;

typedef struct {
    opus_int16                   pred_prev_Q13[ 2 ];
    opus_int16                   sMid[ 2 ];
    opus_int16                   sSide[ 2 ];
    opus_int32                   mid_side_amp_Q0[ 4 ];
    opus_int16                   smth_width_Q14;
    opus_int16                   width_prev_Q14;
    opus_int16                   silent_side_len;
    opus_int8                    predIx[ MAX_FRAMES_PER_PACKET ][ 2 ][ 3 ];
    opus_int8                    mid_only_flags[ MAX_FRAMES_PER_PACKET ];
} stereo_enc_state;

typedef struct {
    opus_int16                   pred_prev_Q13[ 2 ];
    opus_int16                   sMid[ 2 ];
    opus_int16                   sSide[ 2 ];
} stereo_dec_state;

typedef struct {
    opus_int8                    GainsIndices[ MAX_NB_SUBFR ];
    opus_int8                    LTPIndex[ MAX_NB_SUBFR ];
    opus_int8                    NLSFIndices[ MAX_LPC_ORDER + 1 ];
    opus_int16                   lagIndex;
    opus_int8                    contourIndex;
    opus_int8                    signalType;
    opus_int8                    quantOffsetType;
    opus_int8                    NLSFInterpCoef_Q2;
    opus_int8                    PERIndex;
    opus_int8                    LTP_scaleIndex;
    opus_int8                    Seed;
} SideInfoIndices;

/********************************/
/* Encoder state                */
/********************************/
typedef struct {
    opus_int32                   In_HP_State[ 2 ];                  /*高通滤波器状态 High pass filter state                                           */
    opus_int32                   variable_HP_smth1_Q15;             /* 一阶平滑状态State of first smoother                                          */
    opus_int32                   variable_HP_smth2_Q15;             /* 二阶平滑状态State of second smoother                                         */
    silk_LP_state                sLP;                               /*低通滤波器状态 Low pass filter state                                            */
    silk_VAD_state               sVAD;                              /* 声音检测器状态Voice activity detector state                                    */
    silk_nsq_state               sNSQ;                              /* 噪声整形量化器状态Noise Shape Quantizer State                                      */
    opus_int16                   prev_NLSFq_Q15[ MAX_LPC_ORDER ];   /*先前量化的NLSF矢量 Previously quantized NLSF vector                                 */
    opus_int                     speech_activity_Q8;                /*语音活动水平 Speech activity                                                  */
    opus_int                     allow_bandwidth_switch;            /*允许在处理数据时转换带宽的标志 Flag indicating that switching of internal bandwidth is allowed  */
    opus_int8                    LBRRprevLastGainIndex;
    opus_int8                    prevSignalType;                    //上一帧信号类型，清音0；浊音1
    opus_int                     prevLag;                           //上一帧基频
    opus_int                     pitch_LPC_win_length;               /*基频分析时LPC窗长度    PacketSize_ms > 10时=(20 + 2 << 1)* fs_kHz=24*fs;*/
    opus_int                     max_pitch_lag;                     /*最大基频 Highest possible pitch lag (samples)                             */
    opus_int32                   API_fs_Hz;                         /*传入silk音频采样率 API sampling frequency (Hz)                                      */
    opus_int32                   prev_API_fs_Hz;                    /*仅用于初始化判断是否建立编码器 Previous API sampling frequency (Hz)                             */
    opus_int                     maxInternal_fs_Hz;                 /*最大内部采样率 ，就是指函数处理数据的最大采样率Maximum internal sampling frequency (Hz)                         */
    opus_int                     minInternal_fs_Hz;                 /*最小内部采样率 Minimum internal sampling frequency (Hz)                         */
    opus_int                     desiredInternal_fs_Hz;             /*实际上使用的采样率 Soft request for internal sampling frequency (Hz)                */
    opus_int                     fs_kHz;                            /*传入音频的采样率 Internal sampling frequency (kHz)                                */
    opus_int                     nb_subfr;                          /*子帧数 PacketSize_ms > 10时最大子帧个数就是4个 Number of 5 ms subframes in a frame                              */
    opus_int                     frame_length;                      /*帧长度 PacketSize_ms > 10时20 * fs_kHzFrame length (samples)                                           */
    opus_int                     subfr_length;                      /*子帧长度 PacketSize_ms > 10时=5*fs_khz  Subframe length (samples)                                        */
    opus_int                     ltp_mem_length;                    /*=20* fs_kHz LTP记忆器的长度Length of LTP memory                                             */
    opus_int                     la_pitch;                          /* =2* fs_kHz (注意这里的fs是经过重采样的）提前考虑音高分析，相当于Look-ahead for pitch analysis (samples)                          */
    opus_int                     la_shape;                          /*噪声整形向前看的样本数 Look-ahead for noise shape analysis (samples)                    */
    opus_int                     shapeWinLength;                    /*整形窗长度 Window length for noise shape analysis (samples)                 */
    opus_int32                   TargetRate_bps;                    /*目标比特率 Target bitrate (bps)                                             */
    opus_int                     PacketSize_ms;                     /*每一帧的ms数 Number of milliseconds to put in each packet                     */
    opus_int                     PacketLoss_perc;                   /* 远端测量的丢包率Packet loss rate measured by farend                              */
    opus_int32                   frameCounter;                      //帧计数器
    opus_int                     Complexity;                        /*设置的复杂度 Complexity setting                                               */
    opus_int                     nStatesDelayedDecision;            /* Number of states in delayed decision quantization                */
    opus_int                     useInterpolatedNLSFs;              /* Flag for using NLSF interpolation                                */
    opus_int                     shapingLPCOrder;                   /* Filter order for noise shaping filters                           */
    opus_int                     predictLPCOrder;                   /* Filter order for prediction filters                              */
    opus_int                     pitchEstimationComplexity;         /* Complexity level for pitch estimator                             */
    opus_int                     pitchEstimationLPCOrder;           /* Whitening filter order for pitch estimator                       */
    opus_int32                   pitchEstimationThreshold_Q16;      /* Threshold for pitch estimator                                    */
    opus_int32                   sum_log_gain_Q7;                   /* Cumulative max prediction gain                                   */
    opus_int                     NLSF_MSVQ_Survivors;               /* Number of survivors in NLSF MSVQ                                 */
    opus_int                     first_frame_after_reset;           /* Flag for deactivating NLSF interpolation, pitch prediction       */
    opus_int                     controlled_since_last_payload;     /* Flag for ensuring codec_control only runs once per packet        */
    opus_int                     warping_Q16;                       /* Warping parameter for warped noise shaping                       */
    opus_int                     useCBR;                            /* Flag to enable constant bitrate                                  */
    opus_int                     prefillFlag;                       /*指示仅预填充缓冲区而不编码的标志 Flag to indicate that only buffers are prefilled, no coding      */
    const opus_uint8             *pitch_lag_low_bits_iCDF;          /* Pointer to iCDF table for low bits of pitch lag index            */
    const opus_uint8             *pitch_contour_iCDF;               /*fs_kHz == 8时则=11阶的系数数组，否则=34阶的数组 Pointer to iCDF table for pitch contour index                    */
    const silk_NLSF_CB_struct    *psNLSF_CB;                        /* Pointer to NLSF codebook                                         */
    opus_int                     input_quality_bands_Q15[ VAD_N_BANDS ];
    opus_int                     input_tilt_Q15;
    opus_int                     SNR_dB_Q7;                         /* Quality setting                                                  */

    opus_int8                    VAD_flags[ MAX_FRAMES_PER_PACKET ];
    opus_int8                    LBRR_flag;
    opus_int                     LBRR_flags[ MAX_FRAMES_PER_PACKET ];

    SideInfoIndices              indices;
    opus_int8                    pulses[ MAX_FRAME_LENGTH ];

    int                          arch;

    /* Input/output buffering */
    opus_int16                   inputBuf[ MAX_FRAME_LENGTH + 2 ];  /* Buffer containing input signal                                   */
    opus_int                     inputBufIx;
    opus_int                     nFramesPerPacket;
    opus_int                     nFramesEncoded;                    /* Number of frames analyzed in current packet                      */

    opus_int                     nChannelsAPI;
    opus_int                     nChannelsInternal;
    opus_int                     channelNb;

    /* Parameters For LTP scaling Control */
    opus_int                     frames_since_onset;

    /* Specifically for entropy coding */
    opus_int                     ec_prevSignalType;
    opus_int16                   ec_prevLagIndex;

    silk_resampler_state_struct resampler_state;

    /* DTX */
    opus_int                     useDTX;                            /* 使用DTX的标志Flag to enable DTX                                               */
    opus_int                     inDTX;                             /*信号DTX周期的标志 Flag to signal DTX period                                        */
    opus_int                     noSpeechCounter;                   /*计数DTX使用的连续非活动帧 Counts concecutive nonactive frames, used by DTX                 */

    /* Inband Low Bitrate Redundancy (LBRR) data */
    opus_int                     useInBandFEC;                      /* 保存API设置以进行查询， 强制启用带内前向纠错(FEC)的标志Saves the API setting for query                                  */
    opus_int                     LBRR_enabled;                      /* Depends on useInBandFRC, bitrate and packet loss rate            */
    opus_int                     LBRR_GainIncreases;                /* Gains increment for coding LBRR frames                           */
    SideInfoIndices              indices_LBRR[ MAX_FRAMES_PER_PACKET ];
    opus_int8                    pulses_LBRR[ MAX_FRAMES_PER_PACKET ][ MAX_FRAME_LENGTH ];
} silk_encoder_state;


/* Struct for Packet Loss Concealment */
typedef struct {
    opus_int32                  pitchL_Q8;                          /* Pitch lag to use for voiced concealment                          */
    opus_int16                  LTPCoef_Q14[ LTP_ORDER ];           /* LTP coeficients to use for voiced concealment                    */
    opus_int16                  prevLPC_Q12[ MAX_LPC_ORDER ];
    opus_int                    last_frame_lost;                    /* Was previous frame lost                                          */
    opus_int32                  rand_seed;                          /* Seed for unvoiced signal generation                              */
    opus_int16                  randScale_Q14;                      /* Scaling of unvoiced random signal                                */
    opus_int32                  conc_energy;
    opus_int                    conc_energy_shift;
    opus_int16                  prevLTP_scale_Q14;
    opus_int32                  prevGain_Q16[ 2 ];
    opus_int                    fs_kHz;
    opus_int                    nb_subfr;
    opus_int                    subfr_length;
} silk_PLC_struct;

/* Struct for CNG */
typedef struct {
    opus_int32                  CNG_exc_buf_Q14[ MAX_FRAME_LENGTH ];
    opus_int16                  CNG_smth_NLSF_Q15[ MAX_LPC_ORDER ];
    opus_int32                  CNG_synth_state[ MAX_LPC_ORDER ];
    opus_int32                  CNG_smth_Gain_Q16;
    opus_int32                  rand_seed;
    opus_int                    fs_kHz;
} silk_CNG_struct;

/********************************/
/* Decoder state                */
/********************************/
typedef struct {
    opus_int32                  prev_gain_Q16;
    opus_int32                  exc_Q14[ MAX_FRAME_LENGTH ];
    opus_int32                  sLPC_Q14_buf[ MAX_LPC_ORDER ];
    opus_int16                  outBuf[ MAX_FRAME_LENGTH + 2 * MAX_SUB_FRAME_LENGTH ];  /* Buffer for output signal                     */
    opus_int                    lagPrev;                            /* Previous Lag                                                     */
    opus_int8                   LastGainIndex;                      /* Previous gain index                                              */
    opus_int                    fs_kHz;                             /* Sampling frequency in kHz                                        */
    opus_int32                  fs_API_hz;                          /* API sample frequency (Hz)                                        */
    opus_int                    nb_subfr;                           /* Number of 5 ms subframes in a frame                              */
    opus_int                    frame_length;                       /* Frame length (samples)                                           */
    opus_int                    subfr_length;                       /* Subframe length (samples)                                        */
    opus_int                    ltp_mem_length;                     /* Length of LTP memory                                             */
    opus_int                    LPC_order;                          /* LPC order                                                        */
    opus_int16                  prevNLSF_Q15[ MAX_LPC_ORDER ];      /* Used to interpolate LSFs                                         */
    opus_int                    first_frame_after_reset;            /* Flag for deactivating NLSF interpolation                         */
    const opus_uint8            *pitch_lag_low_bits_iCDF;           /* Pointer to iCDF table for low bits of pitch lag index            */
    const opus_uint8            *pitch_contour_iCDF;                /* Pointer to iCDF table for pitch contour index                    */

    /* For buffering payload in case of more frames per packet */
    opus_int                    nFramesDecoded;
    opus_int                    nFramesPerPacket;

    /* Specifically for entropy coding */
    opus_int                    ec_prevSignalType;
    opus_int16                  ec_prevLagIndex;

    opus_int                    VAD_flags[ MAX_FRAMES_PER_PACKET ];
    opus_int                    LBRR_flag;
    opus_int                    LBRR_flags[ MAX_FRAMES_PER_PACKET ];

    silk_resampler_state_struct resampler_state;

    const silk_NLSF_CB_struct   *psNLSF_CB;                         /* Pointer to NLSF codebook                                         */

    /* Quantization indices */
    SideInfoIndices             indices;

    /* CNG state */
    silk_CNG_struct             sCNG;

    /* Stuff used for PLC */
    opus_int                    lossCnt;
    opus_int                    prevSignalType;
    int                         arch;

    silk_PLC_struct sPLC;

} silk_decoder_state;

/************************/
/* Decoder control      */
/************************/
typedef struct {
    /* Prediction and coding parameters */
    opus_int                    pitchL[ MAX_NB_SUBFR ];
    opus_int32                  Gains_Q16[ MAX_NB_SUBFR ];
    /* Holds interpolated and final coefficients, 4-byte aligned */
    silk_DWORD_ALIGN opus_int16 PredCoef_Q12[ 2 ][ MAX_LPC_ORDER ];
    opus_int16                  LTPCoef_Q14[ LTP_ORDER * MAX_NB_SUBFR ];
    opus_int                    LTP_scale_Q14;
} silk_decoder_control;


#ifdef __cplusplus
}
#endif

#endif
