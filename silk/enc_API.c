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
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "define.h"
#include "API.h"
#include "control.h"
#include "typedef.h"
#include "stack_alloc.h"
#include "structs.h"
#include "tuning_parameters.h"
#ifdef FIXED_POINT
#include "fixed/main_FIX.h"
#else
#include "float/main_FLP.h"
#endif

/***************************************/
/* Read control structure from encoder */
/***************************************/
static opus_int silk_QueryEncoder(                      /* O    Returns error code                              */
    const void                      *encState,          /* I    State                                           */
    silk_EncControlStruct           *encStatus          /* O    Encoder Status                                  */
);

/****************************************/
/* Encoder functions                    */
/****************************************/

opus_int silk_Get_Encoder_Size(                         /* O    Returns error code                              */
    opus_int                        *encSizeBytes       /* O    Number of bytes in SILK encoder state           */
)
{
    opus_int ret = SILK_NO_ERROR;

    *encSizeBytes = sizeof( silk_encoder );//sizeof( silk_encoder )=17384字节

    return ret;
}

/*************************/
/* Init or Reset encoder */
/*************************/
opus_int silk_InitEncoder(                              /* O    Returns error code                              */
    void                            *encState,          /* I/O  State                                           */
    int                              arch,              /* I    Run-time architecture                           */
    silk_EncControlStruct           *encStatus          /* O    Encoder Status                                  */
)
{
    //=silk_enc指针  就是编码器状态
    silk_encoder *psEnc;
    opus_int n, ret = SILK_NO_ERROR;

    psEnc = (silk_encoder *)encState;

    /* 重置编码器 Reset encoder */
    silk_memset( psEnc, 0, sizeof( silk_encoder ) );//silk_enc置零
    for( n = 0; n < ENCODER_NUM_CHANNELS; n++ ) {
        if( ret += silk_init_encoder( &psEnc->state_Fxx[ n ], arch ) ) {
            celt_assert( 0 );
        }
    }

    psEnc->nChannelsAPI = 1;        //在silk内部函数间音频都是单声道处理
    psEnc->nChannelsInternal = 1;   //函数内部处理音频数据都是单声道             这里强制使得opus的输出只能是单声道

    /* 读取控制结构 Read control structure */
    if( ret += silk_QueryEncoder( encState, encStatus ) ) {//将silk状态器的部分状态（下图）赋值给控制器,这里只是将API和物理声道数赋值给silk控制器
        celt_assert( 0 );
    }

    return ret;
}

/***************************************/
/* Read control structure from encoder */
/***************************************/
static opus_int silk_QueryEncoder(                      /* O    Returns error code                              */
    const void                      *encState,          /* I    State        状态器                                   */
    silk_EncControlStruct           *encStatus          /* O    Encoder Status   控制器                               */
)
{
    opus_int ret = SILK_NO_ERROR;
    silk_encoder_state_Fxx *state_Fxx;
    silk_encoder *psEnc = (silk_encoder *)encState;
    
    state_Fxx = psEnc->state_Fxx;

    encStatus->nChannelsAPI              = psEnc->nChannelsAPI;//API声道数，传入opus的音频声道数已经设置为1
    encStatus->nChannelsInternal         = psEnc->nChannelsInternal;//物理声道数，opus处理数据的声道数，已经设置为1
    encStatus->API_sampleRate            = state_Fxx[ 0 ].sCmn.API_fs_Hz;//传入opus的音频采样率大小
    encStatus->maxInternalSampleRate     = state_Fxx[ 0 ].sCmn.maxInternal_fs_Hz;//处理数据时最大采样率
    encStatus->minInternalSampleRate     = state_Fxx[ 0 ].sCmn.minInternal_fs_Hz;//处理数据时最小采样率
    encStatus->desiredInternalSampleRate = state_Fxx[ 0 ].sCmn.desiredInternal_fs_Hz;//处理数据时实际采样率
    encStatus->payloadSize_ms            = state_Fxx[ 0 ].sCmn.PacketSize_ms;//每帧的时长，和framesize相关
    encStatus->bitRate                   = state_Fxx[ 0 ].sCmn.TargetRate_bps;//目标比特率
    encStatus->packetLossPercentage      = state_Fxx[ 0 ].sCmn.PacketLoss_perc;//上行数据丢包率百分比
    encStatus->complexity                = state_Fxx[ 0 ].sCmn.Complexity;//设置的复杂度
    encStatus->useInBandFEC              = state_Fxx[ 0 ].sCmn.useInBandFEC;// 强制启用带内前向纠错(FEC)的标志
    encStatus->useDTX                    = state_Fxx[ 0 ].sCmn.useDTX;//使用DTX标志
    encStatus->useCBR                    = state_Fxx[ 0 ].sCmn.useCBR;//使用CBR标志
    encStatus->internalSampleRate        = silk_SMULBB( state_Fxx[ 0 ].sCmn.fs_kHz, 1000 );//内部采样率hz
    encStatus->allowBandwidthSwitch      = state_Fxx[ 0 ].sCmn.allow_bandwidth_switch;//允许带宽转换
    encStatus->inWBmodeWithoutVariableLP = state_Fxx[ 0 ].sCmn.fs_kHz == 16 && state_Fxx[ 0 ].sCmn.sLP.mode == 0;//只有在输入音频采样率等于16k且不采用mode情况下才使用不可变的LP

    return ret;
}


/**************************/
/* Encode frame with Silk */
/**************************/
/* Note: if prefillFlag is set, the input must contain 10 ms of audio, irrespective of what                     */
/* encControl->payloadSize_ms is set to                                                                         */
opus_int silk_Encode(                                   /* O    Returns error code                              */
    void                            *encState,          /* I/O  编码状态                                    */
    silk_EncControlStruct           *encControl,        /* I    编码控制状态                                  */
    const opus_int16                *samplesIn,         /* I    输入语音，即测试程序opus_demo读取到的PCM，是vector容器内的数据                     */
    opus_int                        nSamplesIn,         /* I    采样点数480               */
    ec_enc                          *psRangeEnc,        /* I/O  区间编码压缩结构体                       */
    opus_int32                      *nBytesOut,         /* I/O  区间编码数据长度(input: Max bytes)   */
    const opus_int                  prefillFlag,        /* I    指示prefilling buffer数据是否编码，置位后表示不参与编码，测试情况没有设置该位   */
    opus_int                        activity            /* I    语音检测概率Decision of Opus voice activity detector        */
)
{
    opus_int   n, i, nBits, flags, tmp_payloadSize_ms = 0, tmp_complexity = 0, ret = 0;
    opus_int   nSamplesToBuffer;                                                                              //
    opus_int   nSamplesToBufferMax;                                                                           //
    opus_int   nBlocksOf10ms;
    opus_int   nSamplesFromInput = 0, nSamplesFromInputMax;
    opus_int   speech_act_thr_for_switch_Q8;
    opus_int32 TargetRate_bps, MStargetRates_bps[ 2 ], channelRate_bps, LBRR_symbol, sum;
    silk_encoder *psEnc = ( silk_encoder * )encState;//编码状态
    VARDECL( opus_int16, buf );
    opus_int transition, curr_block, tot_blocks;//tot_blocks=1，curr_block=0，transition=0
    SAVE_STACK;
    if (encControl->reducedDependency)//如果开启降低依赖性的选项的话，将编码器状态设置成对应的模式，实际不进入
    {
       psEnc->state_Fxx[0].sCmn.first_frame_after_reset = 1;
       psEnc->state_Fxx[1].sCmn.first_frame_after_reset = 1;
    }
    psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded = psEnc->state_Fxx[ 1 ].sCmn.nFramesEncoded = 0;              //记录编码帧数的参数

    /* 在编码器控制结构中检查数值，采样率以及目标比特率不等于8k12k16k24k32k44.1k48k，最大采样率不为8k12k16k最小采样率不为8k12k16k，最大采样率应大于最小采样率，目标比特率不在最大比特率和最小比特率的范围内Check values in encoder control structure */
    if( ( ret = check_control_input( encControl ) ) != 0 ) {
        celt_assert( 0 );
        RESTORE_STACK;
        return ret;
    }

    encControl->switchReady = 0;
    if( encControl->nChannelsInternal > psEnc->nChannelsInternal ) //1,1不进入
    {
        /*从单声道转换为立体声的过程中，第二声道的初始状态以及立体声的状态 Mono -> Stereo transition: init state of second channel and stereo state */
        ret += silk_init_encoder( &psEnc->state_Fxx[ 1 ], psEnc->state_Fxx[ 0 ].sCmn.arch );
        silk_memset( psEnc->sStereo.pred_prev_Q13, 0, sizeof( psEnc->sStereo.pred_prev_Q13 ) );
        silk_memset( psEnc->sStereo.sSide, 0, sizeof( psEnc->sStereo.sSide ) );
        psEnc->sStereo.mid_side_amp_Q0[ 0 ] = 0;
        psEnc->sStereo.mid_side_amp_Q0[ 1 ] = 1;
        psEnc->sStereo.mid_side_amp_Q0[ 2 ] = 0;
        psEnc->sStereo.mid_side_amp_Q0[ 3 ] = 1;
        psEnc->sStereo.width_prev_Q14 = 0;
        psEnc->sStereo.smth_width_Q14 = SILK_FIX_CONST( 1, 14 );
        if( psEnc->nChannelsAPI == 2 ) {
            silk_memcpy( &psEnc->state_Fxx[ 1 ].sCmn.resampler_state, &psEnc->state_Fxx[ 0 ].sCmn.resampler_state, sizeof( silk_resampler_state_struct ) );
            silk_memcpy( &psEnc->state_Fxx[ 1 ].sCmn.In_HP_State,     &psEnc->state_Fxx[ 0 ].sCmn.In_HP_State,     sizeof( psEnc->state_Fxx[ 1 ].sCmn.In_HP_State ) );
        }
    }

    transition = (encControl->payloadSize_ms != psEnc->state_Fxx[ 0 ].sCmn.PacketSize_ms) || (psEnc->nChannelsInternal != encControl->nChannelsInternal);  //用来判断是否生成lbrr帧,只有第一帧为1

    psEnc->nChannelsAPI = encControl->nChannelsAPI;
    psEnc->nChannelsInternal = encControl->nChannelsInternal;

    nBlocksOf10ms = silk_DIV32( 100 * nSamplesIn, encControl->API_sampleRate );//=6,480*100/8k=6              //计算有多少个10ms帧
    tot_blocks = ( nBlocksOf10ms > 1 ) ? nBlocksOf10ms >> 1 : 1;                                              //通过10ms帧个数分成20ms子帧
    curr_block = 0;                                                                                           //初始化处理块的序号
    if( prefillFlag ) //=0
    {
        /* Only accept input length of 10 ms */
        if( nBlocksOf10ms != 1 ) {
            celt_assert( 0 );
            RESTORE_STACK;
            return SILK_ENC_INPUT_INVALID_NO_OF_SAMPLES;
        }
        /* Reset Encoder */
        for( n = 0; n < encControl->nChannelsInternal; n++ ) {
            ret = silk_init_encoder( &psEnc->state_Fxx[ n ], psEnc->state_Fxx[ n ].sCmn.arch );
            celt_assert( !ret );
        }
        tmp_payloadSize_ms = encControl->payloadSize_ms;
        encControl->payloadSize_ms = 10;
        tmp_complexity = encControl->complexity;
        encControl->complexity = 0;
        for( n = 0; n < encControl->nChannelsInternal; n++ ) {
            psEnc->state_Fxx[ n ].sCmn.controlled_since_last_payload = 0;
            psEnc->state_Fxx[ n ].sCmn.prefillFlag = 1;
        }
    } 
    else {
        /* 只接受输入长度为10ms的倍数的数据Only accept input lengths that are a multiple of 10 ms */
        if( nBlocksOf10ms * encControl->API_sampleRate != 100 * nSamplesIn || nSamplesIn < 0 ) {
            celt_assert( 0 );
            RESTORE_STACK;
            return SILK_ENC_INPUT_INVALID_NO_OF_SAMPLES;
        }
        /* 确保最多只能生成一个数据包Make sure no more than one packet can be produced */
        if( 1000 * (opus_int32)nSamplesIn > encControl->payloadSize_ms * encControl->API_sampleRate ) {
            celt_assert( 0 );
            RESTORE_STACK;
            return SILK_ENC_INPUT_INVALID_NO_OF_SAMPLES;
        }
    }
    for( n = 0; n < encControl->nChannelsInternal; n++ ) 
    {
        /* 侧信道的速率强制设定为与中心信道相同的速率Force the side channel to the same rate as the mid */
        opus_int force_fs_kHz = (n==1) ? psEnc->state_Fxx[0].sCmn.fs_kHz : 0;//=8
        if( ( ret = silk_control_encoder( &psEnc->state_Fxx[ n ], encControl, psEnc->allowBandwidthSwitch, n, force_fs_kHz ) ) != 0 ) //在判断语句中完成了设置参数
        {
            silk_assert( 0 );
            RESTORE_STACK;
            return ret;
        }
        if( psEnc->state_Fxx[n].sCmn.first_frame_after_reset || transition ) {                                              //只有第一帧进入此判断
            for( i = 0; i < psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket; i++ ) {
                psEnc->state_Fxx[ n ].sCmn.LBRR_flags[ i ] = 0;
            }
        }
        psEnc->state_Fxx[ n ].sCmn.inDTX = psEnc->state_Fxx[ n ].sCmn.useDTX;
    }
    celt_assert( encControl->nChannelsInternal == 1 || psEnc->state_Fxx[ 0 ].sCmn.fs_kHz == psEnc->state_Fxx[ 1 ].sCmn.fs_kHz );

    /* 对输入信号进行缓冲或重新采样，然后编码Input buffering/resampling and encoding */
    nSamplesToBufferMax =10 * nBlocksOf10ms * psEnc->state_Fxx[ 0 ].sCmn.fs_kHz;//=480
    nSamplesFromInputMax = silk_DIV32_16(nSamplesToBufferMax * psEnc->state_Fxx[0].sCmn.API_fs_Hz, psEnc->state_Fxx[0].sCmn.fs_kHz * 1000);//=480
    ALLOC( buf, nSamplesFromInputMax, opus_int16 );
    while( 1 ) {
        nSamplesToBuffer  = psEnc->state_Fxx[ 0 ].sCmn.frame_length - psEnc->state_Fxx[ 0 ].sCmn.inputBufIx;//160
        nSamplesToBuffer  = silk_min( nSamplesToBuffer, nSamplesToBufferMax );//160
        nSamplesFromInput = silk_DIV32_16( nSamplesToBuffer * psEnc->state_Fxx[ 0 ].sCmn.API_fs_Hz, psEnc->state_Fxx[ 0 ].sCmn.fs_kHz * 1000 );     //输入样本数，分成子帧输入
        /* 重采样并写到缓冲区Resample and write to buffer */
        if( encControl->nChannelsAPI == 2 && encControl->nChannelsInternal == 2 ) //对立体声重采样，不进入
        {
            opus_int id = psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded;
            for( n = 0; n < nSamplesFromInput; n++ ) {
                buf[ n ] = samplesIn[ 2 * n ];
            }
            /* Making sure to start both resamplers from the same state when switching from mono to stereo */
            if( psEnc->nPrevChannelsInternal == 1 && id==0 ) {
               silk_memcpy( &psEnc->state_Fxx[ 1 ].sCmn.resampler_state, &psEnc->state_Fxx[ 0 ].sCmn.resampler_state, sizeof(psEnc->state_Fxx[ 1 ].sCmn.resampler_state));
            }

            ret += silk_resampler( &psEnc->state_Fxx[ 0 ].sCmn.resampler_state,
                &psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.inputBufIx + 2 ], buf, nSamplesFromInput );
            //inputBuf用于存放重采样之后的数据，重采样的状态是放在resampler_state变量里的。
            psEnc->state_Fxx[ 0 ].sCmn.inputBufIx += nSamplesToBuffer;

            nSamplesToBuffer  = psEnc->state_Fxx[ 1 ].sCmn.frame_length - psEnc->state_Fxx[ 1 ].sCmn.inputBufIx;
            nSamplesToBuffer  = silk_min( nSamplesToBuffer, 10 * nBlocksOf10ms * psEnc->state_Fxx[ 1 ].sCmn.fs_kHz );
            for( n = 0; n < nSamplesFromInput; n++ ) {
                buf[ n ] = samplesIn[ 2 * n + 1 ];
            }
            ret += silk_resampler( &psEnc->state_Fxx[ 1 ].sCmn.resampler_state,
                &psEnc->state_Fxx[ 1 ].sCmn.inputBuf[ psEnc->state_Fxx[ 1 ].sCmn.inputBufIx + 2 ], buf, nSamplesFromInput );

            psEnc->state_Fxx[ 1 ].sCmn.inputBufIx += nSamplesToBuffer;
        } 
        else 
            if( encControl->nChannelsAPI == 2 && encControl->nChannelsInternal == 1 ) {
            /* Combine left and right channels before resampling */
            for( n = 0; n < nSamplesFromInput; n++ ) {
                sum = samplesIn[ 2 * n ] + samplesIn[ 2 * n + 1 ];
                buf[ n ] = (opus_int16)silk_RSHIFT_ROUND( sum,  1 );
            }
            ret += silk_resampler( &psEnc->state_Fxx[ 0 ].sCmn.resampler_state,
                &psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.inputBufIx + 2 ], buf, nSamplesFromInput );
            /* On the first mono frame, average the results for the two resampler states  */
            if( psEnc->nPrevChannelsInternal == 2 && psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded == 0 ) {
               ret += silk_resampler( &psEnc->state_Fxx[ 1 ].sCmn.resampler_state,
                   &psEnc->state_Fxx[ 1 ].sCmn.inputBuf[ psEnc->state_Fxx[ 1 ].sCmn.inputBufIx + 2 ], buf, nSamplesFromInput );
               for( n = 0; n < psEnc->state_Fxx[ 0 ].sCmn.frame_length; n++ ) {
                  psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.inputBufIx+n+2 ] =
                        silk_RSHIFT(psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.inputBufIx+n+2 ]
                                  + psEnc->state_Fxx[ 1 ].sCmn.inputBuf[ psEnc->state_Fxx[ 1 ].sCmn.inputBufIx+n+2 ], 1);
               }
            }
            psEnc->state_Fxx[ 0 ].sCmn.inputBufIx += nSamplesToBuffer;
        } 
            else {
            celt_assert( encControl->nChannelsAPI == 1 && encControl->nChannelsInternal == 1 );                                            //检查通道数是否正确
            silk_memcpy(buf, samplesIn, nSamplesFromInput*sizeof(opus_int16));                                                             //将输入数据samplesIn拷贝到buf缓冲区，传入一帧
            ret += silk_resampler( &psEnc->state_Fxx[ 0 ].sCmn.resampler_state,&psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.inputBufIx + 2 ], buf, nSamplesFromInput );
                                                                                                                                           //将buf中数据进行重采样到inputbuf中
            psEnc->state_Fxx[ 0 ].sCmn.inputBufIx += nSamplesToBuffer;                                                                     //更新记录数据长度的inputBufIx变量
        }

        samplesIn  += nSamplesFromInput * encControl->nChannelsAPI;                                                                        //传入地址增加一子帧
        nSamplesIn -= nSamplesFromInput;                                                                                                   //还剩下的样本数

        /* Default */
        psEnc->allowBandwidthSwitch = 0;                                                                                                   //默认带宽不变化

        /* Silk encoder */
        if( psEnc->state_Fxx[ 0 ].sCmn.inputBufIx >= psEnc->state_Fxx[ 0 ].sCmn.frame_length )//160,160
        {
            /*数据足够编码，检查参数 Enough data in input buffer, so encode */
            celt_assert( psEnc->state_Fxx[ 0 ].sCmn.inputBufIx == psEnc->state_Fxx[ 0 ].sCmn.frame_length );//检查音频编码器数据输入缓冲区的系数是否与音频帧长度相同。
            celt_assert( encControl->nChannelsInternal == 1 || psEnc->state_Fxx[ 1 ].sCmn.inputBufIx == psEnc->state_Fxx[ 1 ].sCmn.frame_length );

            /* Deal with LBRR data 0,1,2,一共分为三帧*/
            if( psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded == 0 && !prefillFlag ) {
                /* 给VAD和FEC标志创造空间Create space at start of payload for VAD and FEC flags */
                opus_uint8 iCDF[ 2 ] = { 0, 0 };
                iCDF[ 0 ] = 256 - silk_RSHIFT( 256, ( psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket + 1 ) * encControl->nChannelsInternal );
                ec_enc_icdf( psRangeEnc, 0, iCDF, 8 );//

                /* Encode any LBRR data from previous packet */
                /* Encode LBRR flags */
                for( n = 0; n < encControl->nChannelsInternal; n++ )  //走三次
                {
                    LBRR_symbol = 0;
                    for( i = 0; i < psEnc->state_Fxx[ n ].sCmn.nFramesPerPacket; i++ ) {
                        LBRR_symbol |= silk_LSHIFT( psEnc->state_Fxx[ n ].sCmn.LBRR_flags[ i ], i );//0，0，0
                    }
                    psEnc->state_Fxx[ n ].sCmn.LBRR_flag = LBRR_symbol > 0 ? 1 : 0;//=0
                    if( LBRR_symbol && psEnc->state_Fxx[ n ].sCmn.nFramesPerPacket > 1 ) {
                        ec_enc_icdf( psRangeEnc, LBRR_symbol - 1, silk_LBRR_flags_iCDF_ptr[ psEnc->state_Fxx[ n ].sCmn.nFramesPerPacket - 2 ], 8 );
                    }
                }

                /* 编码LBRR指标和激励信号Code LBRR indices and excitation signals */
                for( i = 0; i < psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket; i++ ) {
                    for( n = 0; n < encControl->nChannelsInternal; n++ ) {
                        if( psEnc->state_Fxx[ n ].sCmn.LBRR_flags[ i ] ) {
                            opus_int condCoding;
                            if( encControl->nChannelsInternal == 2 && n == 0 ) {
                                silk_stereo_encode_pred( psRangeEnc, psEnc->sStereo.predIx[ i ] );
                                /* 对于LBRR数据，如果设置了侧信道LBRR标志，则无需对仅中间标志进行编码For LBRR data there's no need to code the mid-only flag if the side-channel LBRR flag is set */
                                if( psEnc->state_Fxx[ 1 ].sCmn.LBRR_flags[ i ] == 0 ) {
                                    silk_stereo_encode_mid_only( psRangeEnc, psEnc->sStereo.mid_only_flags[ i ] );
                                }
                            }
                            /* 如果前一帧可用，则使用条件编码Use conditional coding if previous frame available */
                            if( i > 0 && psEnc->state_Fxx[ n ].sCmn.LBRR_flags[ i - 1 ] ) {
                                condCoding = CODE_CONDITIONALLY;
                            } else {
                                condCoding = CODE_INDEPENDENTLY;
                            }
                            silk_encode_indices( &psEnc->state_Fxx[ n ].sCmn, psRangeEnc, i, 1, condCoding );
                            silk_encode_pulses( psRangeEnc, psEnc->state_Fxx[ n ].sCmn.indices_LBRR[i].signalType, psEnc->state_Fxx[ n ].sCmn.indices_LBRR[i].quantOffsetType,
                                psEnc->state_Fxx[ n ].sCmn.pulses_LBRR[ i ], psEnc->state_Fxx[ n ].sCmn.frame_length );
                        }
                    }
                }

                /* 重置lbrr标志 */
                for( n = 0; n < encControl->nChannelsInternal; n++ ) {
                    silk_memset( psEnc->state_Fxx[ n ].sCmn.LBRR_flags, 0, sizeof( psEnc->state_Fxx[ n ].sCmn.LBRR_flags ) );
                }
                 
                psEnc->nBitsUsedLBRR = ec_tell( psRangeEnc );//=5
            }

            silk_HP_variable_cutoff( psEnc->state_Fxx );                                                                                //如果上一帧是浊音则更新高通滤波器的最低频率

            /* 数据包的总目标位是实际比特率*一帧的大小msTotal target bits for packet */
            nBits = silk_DIV32_16( silk_MUL( encControl->bitRate, encControl->payloadSize_ms ), 1000 );
            /* 减去用于LBRR的位Subtract bits used for LBRR */
            if( !prefillFlag ) {
                nBits -= psEnc->nBitsUsedLBRR;
            }
            /* 除以数据包中剩余的未编码帧数Divide by number of uncoded frames left in packet */
            nBits = silk_DIV32_16( nBits, psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket );
            /* Convert to bits/second */
            if( encControl->payloadSize_ms == 10 ) //不走if，payloadSize_ms =60
            {
                TargetRate_bps = silk_SMULBB( nBits, 100 );
            } else {
                TargetRate_bps = silk_SMULBB( nBits, 50 );//8350
            }
            /* 减去前一帧和数据包中超过目标的比特部分Subtract fraction of bits in excess of target in previous frames and packets */
            TargetRate_bps -= silk_DIV32_16( silk_MUL( psEnc->nBitsExceeded, 1000 ), BITRESERVOIR_DECAY_TIME_MS );//=8350
            if( !prefillFlag && psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded > 0 ) //不进入prefillFlag=0，nFramesEncoded=0现在没有编码
            {
                /* 比较此数据包中迄今为止的实际比特与目标比特Compare actual vs target bits so far in this packet */
                opus_int32 bitsBalance = ec_tell( psRangeEnc ) - psEnc->nBitsUsedLBRR - nBits * psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded;
                TargetRate_bps -= silk_DIV32_16( silk_MUL( bitsBalance, 1000 ), BITRESERVOIR_DECAY_TIME_MS );
            }
            /* 永远不要超过输入比特率Never exceed input bitrate */
            TargetRate_bps = silk_LIMIT( TargetRate_bps, encControl->bitRate, 5000 );

            /* /将左/右转换为中/侧不进入，输入为单声道Convert Left/Right to Mid/Side */
            if( encControl->nChannelsInternal == 2 ) {
                silk_stereo_LR_to_MS( &psEnc->sStereo, &psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ 2 ], &psEnc->state_Fxx[ 1 ].sCmn.inputBuf[ 2 ],
                    psEnc->sStereo.predIx[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ], &psEnc->sStereo.mid_only_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ],
                    MStargetRates_bps, TargetRate_bps, psEnc->state_Fxx[ 0 ].sCmn.speech_activity_Q8, encControl->toMono,
                    psEnc->state_Fxx[ 0 ].sCmn.fs_kHz, psEnc->state_Fxx[ 0 ].sCmn.frame_length );
                if( psEnc->sStereo.mid_only_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ] == 0 ) {
                    /* Reset side channel encoder memory for first frame with side coding */
                    if( psEnc->prev_decode_only_middle == 1 ) {
                        silk_memset( &psEnc->state_Fxx[ 1 ].sShape,               0, sizeof( psEnc->state_Fxx[ 1 ].sShape ) );
                        silk_memset( &psEnc->state_Fxx[ 1 ].sCmn.sNSQ,            0, sizeof( psEnc->state_Fxx[ 1 ].sCmn.sNSQ ) );
                        silk_memset( psEnc->state_Fxx[ 1 ].sCmn.prev_NLSFq_Q15,   0, sizeof( psEnc->state_Fxx[ 1 ].sCmn.prev_NLSFq_Q15 ) );
                        silk_memset( &psEnc->state_Fxx[ 1 ].sCmn.sLP.In_LP_State, 0, sizeof( psEnc->state_Fxx[ 1 ].sCmn.sLP.In_LP_State ) );
                        psEnc->state_Fxx[ 1 ].sCmn.prevLag                 = 100;
                        psEnc->state_Fxx[ 1 ].sCmn.sNSQ.lagPrev            = 100;
                        psEnc->state_Fxx[ 1 ].sShape.LastGainIndex         = 10;
                        psEnc->state_Fxx[ 1 ].sCmn.prevSignalType          = TYPE_NO_VOICE_ACTIVITY;
                        psEnc->state_Fxx[ 1 ].sCmn.sNSQ.prev_gain_Q16      = 65536;
                        psEnc->state_Fxx[ 1 ].sCmn.first_frame_after_reset = 1;
                    }
                    silk_encode_do_VAD_Fxx( &psEnc->state_Fxx[ 1 ], activity );
                } else {
                    psEnc->state_Fxx[ 1 ].sCmn.VAD_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ] = 0;
                }
                if( !prefillFlag ) {
                    silk_stereo_encode_pred( psRangeEnc, psEnc->sStereo.predIx[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ] );
                    if( psEnc->state_Fxx[ 1 ].sCmn.VAD_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ] == 0 ) {
                        silk_stereo_encode_mid_only( psRangeEnc, psEnc->sStereo.mid_only_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded ] );
                    }
                }
            } 
            else {
                /* 缓存Buffering */
                silk_memcpy( psEnc->state_Fxx[ 0 ].sCmn.inputBuf, psEnc->sStereo.sMid, 2 * sizeof( opus_int16 ) );                   //将smid前两个数传给inputbuf
                silk_memcpy( psEnc->sStereo.sMid, &psEnc->state_Fxx[ 0 ].sCmn.inputBuf[ psEnc->state_Fxx[ 0 ].sCmn.frame_length ], 2 * sizeof( opus_int16 ) );//将下一子帧的前两个数传给smid
            } 
            /*VAD*/
            silk_encode_do_VAD_Fxx( &psEnc->state_Fxx[ 0 ], activity );
            //根据内部函数计算的语音概率判断是否有语音活动，并计算了一些参数用于接下来的计算（每个子带的能量，噪声水平，信噪比，均方根，谱倾斜，语音概率估计）
           
            /* 编码 Encode */
            for( n = 0; n < encControl->nChannelsInternal; n++ ) {
                opus_int maxBits, useCBR;

                /*处理速率限制 Handling rate constraints */
                maxBits = encControl->maxBits;//10200
                if( tot_blocks == 2 && curr_block == 0 ) {
                    maxBits = maxBits * 3 / 5;
                } else if( tot_blocks == 3 ) {
                    if( curr_block == 0 ) {
                        maxBits = maxBits * 2 / 5;//一共有三个块，且当前块是第一块则maxBits=4080
                    } else if( curr_block == 1 ) {
                        maxBits = maxBits * 3 / 4;
                    }
                }
                useCBR = encControl->useCBR && curr_block == tot_blocks - 1;//CBR受预设参数和块数决定，只有预设CBR=1时最后一个块才会使用CBR

                if( encControl->nChannelsInternal == 1 ) {
                    channelRate_bps = TargetRate_bps;//8350
                } else {
                    channelRate_bps = MStargetRates_bps[ n ];
                    if( n == 0 && MStargetRates_bps[ 1 ] > 0 ) {
                        useCBR = 0;
                        /* Give mid up to 1/2 of the max bits for that frame */
                        maxBits -= encControl->maxBits / ( tot_blocks * 2 );
                    }
                }
                                                                                                     //上述代码完成了计算目标比特率和输出最大字节
                if( channelRate_bps > 0 ) {
                    opus_int condCoding;//编码状态标志

                    silk_control_SNR( &psEnc->state_Fxx[ n ].sCmn, channelRate_bps );//将输入的比特率转化成信噪比，方式是通过输入采样率查表线性映射，输入比特率8350，转化完SNR_dB_Q7 =2000

                    /* 如果没有可用的前一帧，请使用独立编码Use independent coding if no previous frame available */
                    if( psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded - n <= 0 ) {
                        condCoding = CODE_INDEPENDENTLY;//独立编码标志                                      //独立编码标志在计算线性预测参数之后才有用
                    } else if( n > 0 && psEnc->prev_decode_only_middle ) {
                        /* 如果我们跳过了这个数据包中的一个侧帧，我们就不需要LTP缩放；LTP状态是明确定义的。If we skipped a side frame in this packet, we don'tneed LTP scaling; the LTP state is well-defined. */
                        condCoding = CODE_INDEPENDENTLY_NO_LTP_SCALING;//独立且不需要LTP缩放标志
                    } else {
                        condCoding = CODE_CONDITIONALLY;//条件编码
                    }
                    if( ( ret = silk_encode_frame_Fxx( &psEnc->state_Fxx[ n ], nBytesOut, psRangeEnc, condCoding, maxBits, useCBR ) ) != 0 )            //在判断语句中完成了完成子帧编码，完成了声道参数和激励信号的编码压缩
                    { 
                        silk_assert( 0 );
                    }
                }
                psEnc->state_Fxx[ n ].sCmn.controlled_since_last_payload = 0;
                psEnc->state_Fxx[ n ].sCmn.inputBufIx = 0;
                psEnc->state_Fxx[ n ].sCmn.nFramesEncoded++;
            }
            psEnc->prev_decode_only_middle = psEnc->sStereo.mid_only_flags[ psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded - 1 ];

            /* 在比特流的开头插入VAD和FEC标志Insert VAD and FEC flags at beginning of bitstream */
            if( *nBytesOut > 0 && psEnc->state_Fxx[ 0 ].sCmn.nFramesEncoded == psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket)  {
                flags = 0;
                for( n = 0; n < encControl->nChannelsInternal; n++ ) {
                    for( i = 0; i < psEnc->state_Fxx[ n ].sCmn.nFramesPerPacket; i++ ) {
                        flags  = silk_LSHIFT( flags, 1 );
                        flags |= psEnc->state_Fxx[ n ].sCmn.VAD_flags[ i ];
                    }
                    flags  = silk_LSHIFT( flags, 1 );
                    flags |= psEnc->state_Fxx[ n ].sCmn.LBRR_flag;
                }
                if( !prefillFlag ) {
                    ec_enc_patch_initial_bits( psRangeEnc, flags, ( psEnc->state_Fxx[ 0 ].sCmn.nFramesPerPacket + 1 ) * encControl->nChannelsInternal );          //添加便于解压缩感知的比特
                }

                /* 如果所有通道DTXed，则返回零字节Return zero bytes if all channels DTXed */
                if( psEnc->state_Fxx[ 0 ].sCmn.inDTX && ( encControl->nChannelsInternal == 1 || psEnc->state_Fxx[ 1 ].sCmn.inDTX ) ) {
                    *nBytesOut = 0;
                }

                psEnc->nBitsExceeded += *nBytesOut * 8;
                psEnc->nBitsExceeded -= silk_DIV32_16( silk_MUL( encControl->bitRate, encControl->payloadSize_ms ), 1000 );
                psEnc->nBitsExceeded  = silk_LIMIT( psEnc->nBitsExceeded, 0, 10000 );

                /* 指示是否允许带宽切换的更新标志Update flag indicating if bandwidth switching is allowed */
                speech_act_thr_for_switch_Q8 = silk_SMLAWB( SILK_FIX_CONST( SPEECH_ACTIVITY_DTX_THRES, 8 ),
                    SILK_FIX_CONST( ( 1 - SPEECH_ACTIVITY_DTX_THRES ) / MAX_BANDWIDTH_SWITCH_DELAY_MS, 16 + 8 ), psEnc->timeSinceSwitchAllowed_ms );
                if( psEnc->state_Fxx[ 0 ].sCmn.speech_activity_Q8 < speech_act_thr_for_switch_Q8 ) {
                    psEnc->allowBandwidthSwitch = 1;
                    psEnc->timeSinceSwitchAllowed_ms = 0;
                } else {
                    psEnc->allowBandwidthSwitch = 0;
                    psEnc->timeSinceSwitchAllowed_ms += encControl->payloadSize_ms;
                }
            }

            if( nSamplesIn == 0 ) {
                break;
            }
        } 
else {
            break;
        }
        curr_block++;//块数加一
    }
    //
    psEnc->nPrevChannelsInternal = encControl->nChannelsInternal;

    encControl->allowBandwidthSwitch = psEnc->allowBandwidthSwitch;
    encControl->inWBmodeWithoutVariableLP = psEnc->state_Fxx[ 0 ].sCmn.fs_kHz == 16 && psEnc->state_Fxx[ 0 ].sCmn.sLP.mode == 0;
    encControl->internalSampleRate = silk_SMULBB( psEnc->state_Fxx[ 0 ].sCmn.fs_kHz, 1000 );
    encControl->stereoWidth_Q14 = encControl->toMono ? 0 : psEnc->sStereo.smth_width_Q14;
    if( prefillFlag ) {
        encControl->payloadSize_ms = tmp_payloadSize_ms;
        encControl->complexity = tmp_complexity;
        for( n = 0; n < encControl->nChannelsInternal; n++ ) {
            psEnc->state_Fxx[ n ].sCmn.controlled_since_last_payload = 0;
            psEnc->state_Fxx[ n ].sCmn.prefillFlag = 0;
        }
    }

    encControl->signalType = psEnc->state_Fxx[0].sCmn.indices.signalType;
    encControl->offset = silk_Quantization_Offsets_Q10
                         [ psEnc->state_Fxx[0].sCmn.indices.signalType >> 1 ]
                         [ psEnc->state_Fxx[0].sCmn.indices.quantOffsetType ];
    RESTORE_STACK;
    return ret;
}

