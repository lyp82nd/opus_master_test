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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include "main_FIX.h"
#include "stack_alloc.h"
#include "tuning_parameters.h"

/* Low Bitrate Redundancy (LBRR) encoding. Reuse all parameters but encode with lower bitrate           */
static OPUS_INLINE void silk_LBRR_encode_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  Pointer to Silk FIX encoder control struct                                  */
    const opus_int16                x16[],                                  /* I    Input signal                                                                */
    opus_int                        condCoding                              /* I    The type of conditional coding used so far for this frame                   */
);

void silk_encode_do_VAD_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    opus_int                        activity                                /* I    Decision of Opus voice activity detector                                    */
)
{
    const opus_int activity_threshold = SILK_FIX_CONST( SPEECH_ACTIVITY_DTX_THRES, 8 );//=13

    /****************************/
    /* 语音活动检测Voice Activity Detection */
    /****************************/
    silk_VAD_GetSA_Q8( &psEnc->sCmn, psEnc->sCmn.inputBuf + 1, psEnc->sCmn.arch );//语音检测主要函数
    /* 如果Opus VAD处于非活动状态，而Silk VAD处于活动状态：将Silk VAD降低到刚好低于阈值If Opus VAD is inactive and Silk VAD is active: lower Silk VAD to just under the threshold */
    if( activity == VAD_NO_ACTIVITY && psEnc->sCmn.speech_activity_Q8 >= activity_threshold ) // >=13
    {
        psEnc->sCmn.speech_activity_Q8 = activity_threshold - 1;//如果最终计算的结果是opusVAD显示无语音，而silk VAD显示有语音，则以opus的为准，将silk 的VAD设置为恰好小于门限值，
    }

    /**************************************************/
    /* 将语音活动转换为VAD和DTX标志Convert speech activity into VAD and DTX flags */
    /**************************************************/
    //根据VAD的统计结果，计算DTX的状态，连续200ms没有语音则会进入DTX状态，DTX模式下把psEnc->sCmn.VAD_flags标志设置为零，这样就可以不发送编码语音包，但是当持续没有语音时间超过400ms时，会退出DTX状态，编码一个包发送到远端，以便非语言段声音的连续性
    if( psEnc->sCmn.speech_activity_Q8 < activity_threshold ) {
        psEnc->sCmn.indices.signalType = TYPE_NO_VOICE_ACTIVITY;//无语音活动
        psEnc->sCmn.noSpeechCounter++;//记录可以使用DTX的帧数
        if( psEnc->sCmn.noSpeechCounter <= NB_SPEECH_FRAMES_BEFORE_DTX ) {
            psEnc->sCmn.inDTX = 0;//小于200ms，DTX
        } else if( psEnc->sCmn.noSpeechCounter > MAX_CONSECUTIVE_DTX + NB_SPEECH_FRAMES_BEFORE_DTX ) {
            psEnc->sCmn.noSpeechCounter = NB_SPEECH_FRAMES_BEFORE_DTX;//超过600ms，退出DTX状态，不记录当前帧
            psEnc->sCmn.inDTX           = 0;
        }
        psEnc->sCmn.VAD_flags[ psEnc->sCmn.nFramesEncoded ] = 0;//DTX模式下把psEnc->sCmn.VAD_flags标志设置为零
    } else {
        psEnc->sCmn.noSpeechCounter    = 0;
        psEnc->sCmn.inDTX              = 0;
        psEnc->sCmn.indices.signalType = TYPE_UNVOICED;//清音
        psEnc->sCmn.VAD_flags[ psEnc->sCmn.nFramesEncoded ] = 1;//语音活动标志位，置1
    }
}

/****************/
/* 编码帧Encode frame */
/****************/
opus_int silk_encode_frame_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  原始数据会存放在&psEnc->state_Fxx[ 0 ].sCmn.inputBuf指向的地址中，Pointer to Silk FIX encoder state                                           */
    opus_int32                      *pnBytesOut,                            /* O    Pointer to number of payload bytes;                                         */
    ec_enc                          *psRangeEnc,                            /* I/O  compressor data structure                                                   */
    opus_int                        condCoding,                             /* I    The type of conditional coding to use                                       */
    opus_int                        maxBits,                                /* I    If > 0: maximum number of output bits                                       */
    opus_int                        useCBR                                  /* I    Flag to force constant-bitrate operation                                    */
)    //完成声道参数和激励信号的编码压缩
{
    silk_encoder_control_FIX sEncCtrl;
    opus_int     i, iter, maxIter, found_upper, found_lower, ret = 0;
    opus_int16   *x_frame;
    ec_enc       sRangeEnc_copy, sRangeEnc_copy2;
    silk_nsq_state sNSQ_copy, sNSQ_copy2;
    opus_int32   seed_copy, nBits, nBits_lower, nBits_upper, gainMult_lower, gainMult_upper;
    opus_int32   gainsID, gainsID_lower, gainsID_upper;
    opus_int16   gainMult_Q8;
    opus_int16   ec_prevLagIndex_copy;
    opus_int     ec_prevSignalType_copy;
    opus_int8    LastGainIndex_copy2;
    opus_int     gain_lock[ MAX_NB_SUBFR ] = {0};
    opus_int16   best_gain_mult[ MAX_NB_SUBFR ];
    opus_int     best_sum[ MAX_NB_SUBFR ];
    SAVE_STACK;

    /* This is totally unnecessary but many compilers (including gcc) are too dumb to realise it */
    LastGainIndex_copy2 = nBits_lower = nBits_upper = gainMult_lower = gainMult_upper = 0;

    psEnc->sCmn.indices.Seed = psEnc->sCmn.frameCounter++ & 3;//按位与相当于取低两位,先与在++                                     //？
    
    /**************************************************************/
    /* 设置输入指针，并在输入缓冲区中插入帧Set up Input Pointers, and insert frame in input buffer   */
    /*************************************************************/
    /* 要编码的帧的开始start of frame to encode */
    x_frame = psEnc->x_buf + psEnc->sCmn.ltp_mem_length;//=下一子帧的地址   ltp_mem_length=160是长时预测的记忆长度    x_buf的前面部分是长时预测的记忆长度然后是LA_SHAPE_MS * psEnc->sCmn.fs_kHz长度的整形预留时间然后才是当前帧数据

    /***************************************/
    /*确保平滑的带宽转换 Ensure smooth bandwidth transitions */
    /***************************************/
    silk_LP_variable_cutoff( &psEnc->sCmn.sLP, psEnc->sCmn.inputBuf + 1, psEnc->sCmn.frame_length );//没有实际运行此函数,本应该将输入信号进行低通滤波
    //第一个参数是低通滤波器状态，第二个参数是输入信号，滤波后信号也存在该指针指向地址中，第三个参数/帧长，对于8kHz，60ms帧长，其长度为480分为三块20ms帧长=160


    /*******************************************/
    /*将低通滤波后的信号拷贝到x_frame+ LA_SHAPE_MS * psEnc->sCmn.fs_kHz（5ms的整形时间预留）的地址，LA_SHAPE_MS是噪声形状分析的lookahead毫秒数，该宏为5，即向前看5ms，这会引入5ms的编码延迟 Copy new frame to front of input buffer */
    /*******************************************/
    silk_memcpy( x_frame + LA_SHAPE_MS * psEnc->sCmn.fs_kHz, psEnc->sCmn.inputBuf + 1, psEnc->sCmn.frame_length * sizeof( opus_int16 ) );
    //x_frame+5ms（LA时间长度）*比特率（khz）8，                   //将psEnc->sCmn.inputBuf + 1指向的数据赋值给x_frame + LA_SHAPE_MS * psEnc->sCmn.fs_kHz地址处

    if( !psEnc->sCmn.prefillFlag ) {
        VARDECL( opus_int16, res_pitch );
        VARDECL( opus_uint8, ec_buf_copy );
        opus_int16 *res_pitch_frame;//音高帧开始地址

        ALLOC( res_pitch,psEnc->sCmn.la_pitch + psEnc->sCmn.frame_length+ psEnc->sCmn.ltp_mem_length, opus_int16 );
        /*音高LPC开始剩余帧 start of pitch LPC residual frame */
        res_pitch_frame = res_pitch + psEnc->sCmn.ltp_mem_length;//

        /*****************************************/
        /* 基频搜索，找到音高延迟，初始化LPC分析Find pitch lags, initial LPC analysis */
        /*****************************************/
        silk_find_pitch_lags_FIX( psEnc, &sEncCtrl, res_pitch, x_frame - psEnc->sCmn.ltp_mem_length, psEnc->sCmn.arch );


        /************************/
        /*噪声整形 Noise shape analysis */
        /************************/
        silk_noise_shape_analysis_FIX( psEnc, &sEncCtrl, res_pitch_frame, x_frame, psEnc->sCmn.arch );

        /***************************************************/
        /*找到线性预测系数 Find linear prediction coefficients (LPC + LTP) */
        /***************************************************/

        silk_find_pred_coefs_FIX( psEnc, &sEncCtrl, res_pitch_frame, x_frame, condCoding );//注意condCoding标志，独立编码和条件编码有什么不同

        /****************************************/
        /* 处理增益Process gains                        */
        /****************************************/
        silk_process_gains_FIX( psEnc, &sEncCtrl, condCoding );

        /****************************************/
        /*低比特率冗余编码 Low Bitrate Redundant Encoding       */
        /****************************************/
        silk_LBRR_encode_FIX( psEnc, &sEncCtrl, x_frame, condCoding );

        /* 循环量化器和熵编码控制比特率Loop over quantizer and entropy coding to control bitrate */
        maxIter = 6;
        gainMult_Q8 = SILK_FIX_CONST( 1, 8 );
        found_lower = 0;
        found_upper = 0;
        gainsID = silk_gains_ID( psEnc->sCmn.indices.GainsIndices, psEnc->sCmn.nb_subfr );
        gainsID_lower = -1;
        gainsID_upper = -1;
        /* 复制一部分输入状态Copy part of the input state */
        silk_memcpy( &sRangeEnc_copy, psRangeEnc, sizeof( ec_enc ) );
        silk_memcpy( &sNSQ_copy, &psEnc->sCmn.sNSQ, sizeof( silk_nsq_state ) );
        seed_copy = psEnc->sCmn.indices.Seed;
        ec_prevLagIndex_copy = psEnc->sCmn.ec_prevLagIndex;
        ec_prevSignalType_copy = psEnc->sCmn.ec_prevSignalType;
        ALLOC( ec_buf_copy, 1275, opus_uint8 );
        for( iter = 0; ; iter++ ) {
            if( gainsID == gainsID_lower ) {
                nBits = nBits_lower;
            } else if( gainsID == gainsID_upper ) {
                nBits = nBits_upper;
            } else {
                /* 储存一部分输入状态Restore part of the input state */
                if( iter > 0 ) {
                    silk_memcpy( psRangeEnc, &sRangeEnc_copy, sizeof( ec_enc ) );
                    silk_memcpy( &psEnc->sCmn.sNSQ, &sNSQ_copy, sizeof( silk_nsq_state ) );
                    psEnc->sCmn.indices.Seed = seed_copy;
                    psEnc->sCmn.ec_prevLagIndex = ec_prevLagIndex_copy;
                    psEnc->sCmn.ec_prevSignalType = ec_prevSignalType_copy;
                }

                /*****************************************/
                /* 噪声整形量化Noise shaping quantization            */
                /*****************************************/
                if( psEnc->sCmn.nStatesDelayedDecision > 1 || psEnc->sCmn.warping_Q16 > 0 ) {
                    silk_NSQ_del_dec( &psEnc->sCmn, &psEnc->sCmn.sNSQ, &psEnc->sCmn.indices, x_frame, psEnc->sCmn.pulses,
                           sEncCtrl.PredCoef_Q12[ 0 ], sEncCtrl.LTPCoef_Q14, sEncCtrl.AR_Q13, sEncCtrl.HarmShapeGain_Q14,
                           sEncCtrl.Tilt_Q14, sEncCtrl.LF_shp_Q14, sEncCtrl.Gains_Q16, sEncCtrl.pitchL, sEncCtrl.Lambda_Q10, sEncCtrl.LTP_scale_Q14,
                           psEnc->sCmn.arch );
                } else {
                    silk_NSQ( &psEnc->sCmn, &psEnc->sCmn.sNSQ, &psEnc->sCmn.indices, x_frame, psEnc->sCmn.pulses,
                            sEncCtrl.PredCoef_Q12[ 0 ], sEncCtrl.LTPCoef_Q14, sEncCtrl.AR_Q13, sEncCtrl.HarmShapeGain_Q14,
                            sEncCtrl.Tilt_Q14, sEncCtrl.LF_shp_Q14, sEncCtrl.Gains_Q16, sEncCtrl.pitchL, sEncCtrl.Lambda_Q10, sEncCtrl.LTP_scale_Q14,
                            psEnc->sCmn.arch);
                }

                if ( iter == maxIter && !found_lower ) {
                    silk_memcpy( &sRangeEnc_copy2, psRangeEnc, sizeof( ec_enc ) );
                }

                /****************************************/
                /*编码参数 Encode Parameters                    */
                /****************************************/
                silk_encode_indices( &psEnc->sCmn, psRangeEnc, psEnc->sCmn.nFramesEncoded, 0, condCoding );
//FILE* fp = fopen("pulses.bin", "wb");
//if (fp)
//{
//    fwrite(psEnc->sCmn.pulses, sizeof(opus_int8), 320, fp);
//    fclose(fp);
//}
                /****************************************/
                /*编码激励信号 Encode Excitation Signal             */
                /****************************************/
                silk_encode_pulses( psRangeEnc, psEnc->sCmn.indices.signalType, psEnc->sCmn.indices.quantOffsetType,
                    psEnc->sCmn.pulses, psEnc->sCmn.frame_length );

                nBits = ec_tell( psRangeEnc );

                /* 如果我们在最后一次迭代后仍然失败，请进行一些损害控制If we still bust after the last iteration, do some damage control. */
                if ( iter == maxIter && !found_lower && nBits > maxBits ) {
                    silk_memcpy( psRangeEnc, &sRangeEnc_copy2, sizeof( ec_enc ) );

                    /*保持增益和最后一帧一致 Keep gains the same as the last frame. */
                    psEnc->sShape.LastGainIndex = sEncCtrl.lastGainIndexPrev;
                    for ( i = 0; i < psEnc->sCmn.nb_subfr; i++ ) {
                        psEnc->sCmn.indices.GainsIndices[ i ] = 4;
                    }
                    if (condCoding != CODE_CONDITIONALLY) {
                       psEnc->sCmn.indices.GainsIndices[ 0 ] = sEncCtrl.lastGainIndexPrev;
                    }
                    psEnc->sCmn.ec_prevLagIndex = ec_prevLagIndex_copy;
                    psEnc->sCmn.ec_prevSignalType = ec_prevSignalType_copy;
                    /*清除所有脉冲 Clear all pulses. */
                    for ( i = 0; i < psEnc->sCmn.frame_length; i++ ) {
                        psEnc->sCmn.pulses[ i ] = 0;
                    }

                    silk_encode_indices( &psEnc->sCmn, psRangeEnc, psEnc->sCmn.nFramesEncoded, 0, condCoding );

                    silk_encode_pulses( psRangeEnc, psEnc->sCmn.indices.signalType, psEnc->sCmn.indices.quantOffsetType,
                        psEnc->sCmn.pulses, psEnc->sCmn.frame_length );

                    nBits = ec_tell( psRangeEnc );
                }

                if( useCBR == 0 && iter == 0 && nBits <= maxBits ) {
                    break;
                }
            }

            if( iter == maxIter ) {
                if( found_lower && ( gainsID == gainsID_lower || nBits > maxBits ) ) {
                    /*从满足比特率预算的早期迭代中恢复输出状态 Restore output state from earlier iteration that did meet the bitrate budget */
                    silk_memcpy( psRangeEnc, &sRangeEnc_copy2, sizeof( ec_enc ) );
                    celt_assert( sRangeEnc_copy2.offs <= 1275 );
                    silk_memcpy( psRangeEnc->buf, ec_buf_copy, sRangeEnc_copy2.offs );
                    silk_memcpy( &psEnc->sCmn.sNSQ, &sNSQ_copy2, sizeof( silk_nsq_state ) );
                    psEnc->sShape.LastGainIndex = LastGainIndex_copy2;
                }
                break;
            }

            if( nBits > maxBits ) {
                if( found_lower == 0 && iter >= 2 ) {
                    /* 调整量化器的速率/失真权衡，并丢弃以前的“上限”结果Adjust the quantizer's rate/distortion tradeoff and discard previous "upper" results */
                    sEncCtrl.Lambda_Q10 = silk_ADD_RSHIFT32( sEncCtrl.Lambda_Q10, sEncCtrl.Lambda_Q10, 1 );
                    found_upper = 0;
                    gainsID_upper = -1;
                } else {
                    found_upper = 1;
                    nBits_upper = nBits;
                    gainMult_upper = gainMult_Q8;
                    gainsID_upper = gainsID;
                }
            } 
            else if( nBits < maxBits - 5 ) {
                found_lower = 1;
                nBits_lower = nBits;
                gainMult_lower = gainMult_Q8;
                if( gainsID != gainsID_lower ) {
                    gainsID_lower = gainsID;
                    /* 复制一部分输出状态Copy part of the output state */
                    silk_memcpy( &sRangeEnc_copy2, psRangeEnc, sizeof( ec_enc ) );
                    celt_assert( psRangeEnc->offs <= 1275 );
                    silk_memcpy( ec_buf_copy, psRangeEnc->buf, psRangeEnc->offs );
                    silk_memcpy( &sNSQ_copy2, &psEnc->sCmn.sNSQ, sizeof( silk_nsq_state ) );
                    LastGainIndex_copy2 = psEnc->sShape.LastGainIndex;
                }
            } else {
                /*预算的5位以内：足够 Within 5 bits of budget: close enough */
                break;
            }

            if ( !found_lower && nBits > maxBits ) {
                int j;
                for ( i = 0; i < psEnc->sCmn.nb_subfr; i++ ) {
                    int sum=0;
                    for ( j = i*psEnc->sCmn.subfr_length; j < (i+1)*psEnc->sCmn.subfr_length; j++ ) {
                        sum += abs( psEnc->sCmn.pulses[j] );
                    }
                    if ( iter == 0 || (sum < best_sum[i] && !gain_lock[i]) ) {
                        best_sum[i] = sum;
                        best_gain_mult[i] = gainMult_Q8;
                    } else {
                        gain_lock[i] = 1;
                    }
                }
            }
            if( ( found_lower & found_upper ) == 0 ) {
                /* 根据高速率/失真曲线调整增益Adjust gain according to high-rate rate/distortion curve */
                if( nBits > maxBits ) {
                    if (gainMult_Q8 < 16384) {
                        gainMult_Q8 *= 2;
                    } else {
                        gainMult_Q8 = 32767;
                    }
                } else {
                    opus_int32 gain_factor_Q16;
                    gain_factor_Q16 = silk_log2lin( silk_LSHIFT( nBits - maxBits, 7 ) / psEnc->sCmn.frame_length + SILK_FIX_CONST( 16, 7 ) );
                    gainMult_Q8 = silk_SMULWB( gain_factor_Q16, gainMult_Q8 );
                }

            } 
            else {
                /*通过插值调整增益 Adjust gain by interpolating */
                gainMult_Q8 = gainMult_lower + silk_DIV32_16( silk_MUL( gainMult_upper - gainMult_lower, maxBits - nBits_lower ), nBits_upper - nBits_lower );
                /* 新的增益乘法器必须介于旧范围的25%和75%之间（请注意，gainMult_upper<gainMult_lower）New gain multplier must be between 25% and 75% of old range (note that gainMult_upper < gainMult_lower) */
                if( gainMult_Q8 > silk_ADD_RSHIFT32( gainMult_lower, gainMult_upper - gainMult_lower, 2 ) ) {
                    gainMult_Q8 = silk_ADD_RSHIFT32( gainMult_lower, gainMult_upper - gainMult_lower, 2 );
                } else
                if( gainMult_Q8 < silk_SUB_RSHIFT32( gainMult_upper, gainMult_upper - gainMult_lower, 2 ) ) {
                    gainMult_Q8 = silk_SUB_RSHIFT32( gainMult_upper, gainMult_upper - gainMult_lower, 2 );
                }
            }

            for( i = 0; i < psEnc->sCmn.nb_subfr; i++ ) {
                opus_int16 tmp;
                if ( gain_lock[i] ) {
                    tmp = best_gain_mult[i];
                } else {
                    tmp = gainMult_Q8;
                }
                sEncCtrl.Gains_Q16[ i ] = silk_LSHIFT_SAT32( silk_SMULWB( sEncCtrl.GainsUnq_Q16[ i ], tmp ), 8 );
            }

            /* 量化增益Quantize gains */
            psEnc->sShape.LastGainIndex = sEncCtrl.lastGainIndexPrev;
            silk_gains_quant( psEnc->sCmn.indices.GainsIndices, sEncCtrl.Gains_Q16,
                  &psEnc->sShape.LastGainIndex, condCoding == CODE_CONDITIONALLY, psEnc->sCmn.nb_subfr );

            /* 增益向量的唯一标识符Unique identifier of gains vector */
            gainsID = silk_gains_ID( psEnc->sCmn.indices.GainsIndices, psEnc->sCmn.nb_subfr );
        }
    }

    /*更新输入buffer Update input buffer */
    silk_memmove( psEnc->x_buf, &psEnc->x_buf[ psEnc->sCmn.frame_length ],
        ( psEnc->sCmn.ltp_mem_length + LA_SHAPE_MS * psEnc->sCmn.fs_kHz ) * sizeof( opus_int16 ) );

    /*无熵编码退出 Exit without entropy coding */
    if( psEnc->sCmn.prefillFlag ) {
        /* 无有效载荷No payload */
        *pnBytesOut = 0;
        RESTORE_STACK;
        return ret;
    }

    /*给下一帧的参数 Parameters needed for next frame */
    psEnc->sCmn.prevLag        = sEncCtrl.pitchL[ psEnc->sCmn.nb_subfr - 1 ];                       //将基频延迟传递给下一帧，可以计算低通频率
    psEnc->sCmn.prevSignalType = psEnc->sCmn.indices.signalType;

    /****************************************/
    /* 最终确定有效负载Finalize payload                     */
    /****************************************/
    psEnc->sCmn.first_frame_after_reset = 0;
    /* 载荷大小Payload size */
    *pnBytesOut = silk_RSHIFT( ec_tell( psRangeEnc ) + 7, 3 );

    RESTORE_STACK;
    return ret;
}

/* 低比特率冗余（LBRR）编码。重用所有参数，但以较低比特率编码激励Low-Bitrate Redundancy (LBRR) encoding. Reuse all parameters but encode excitation at lower bitrate  */
static OPUS_INLINE void silk_LBRR_encode_FIX(
    silk_encoder_state_FIX          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state                                           */
    silk_encoder_control_FIX        *psEncCtrl,                             /* I/O  Pointer to Silk FIX encoder control struct                                  */
    const opus_int16                x16[],                                  /* I    Input signal                                                                */
    opus_int                        condCoding                              /* I    The type of conditional coding used so far for this frame                   */
)
{
    opus_int32   TempGains_Q16[ MAX_NB_SUBFR ];
    SideInfoIndices *psIndices_LBRR = &psEnc->sCmn.indices_LBRR[ psEnc->sCmn.nFramesEncoded ];
    silk_nsq_state sNSQ_LBRR;

    /*******************************************/
    /* Control use of inband LBRR              */
    /*******************************************/
    if( psEnc->sCmn.LBRR_enabled && psEnc->sCmn.speech_activity_Q8 > SILK_FIX_CONST( LBRR_SPEECH_ACTIVITY_THRES, 8 ) ) {
        psEnc->sCmn.LBRR_flags[ psEnc->sCmn.nFramesEncoded ] = 1;
        printf("LBRR_enabled424\n");///////////////////////////////////////////////////////////////////////////////////////////
        /* Copy noise shaping quantizer state and quantization indices from regular encoding */
        silk_memcpy( &sNSQ_LBRR, &psEnc->sCmn.sNSQ, sizeof( silk_nsq_state ) );
        silk_memcpy( psIndices_LBRR, &psEnc->sCmn.indices, sizeof( SideInfoIndices ) );

        /* Save original gains */
        silk_memcpy( TempGains_Q16, psEncCtrl->Gains_Q16, psEnc->sCmn.nb_subfr * sizeof( opus_int32 ) );

        if( psEnc->sCmn.nFramesEncoded == 0 || psEnc->sCmn.LBRR_flags[ psEnc->sCmn.nFramesEncoded - 1 ] == 0 ) {
            /* First frame in packet or previous frame not LBRR coded */
            psEnc->sCmn.LBRRprevLastGainIndex = psEnc->sShape.LastGainIndex;

            /* Increase Gains to get target LBRR rate */
            psIndices_LBRR->GainsIndices[ 0 ] = psIndices_LBRR->GainsIndices[ 0 ] + psEnc->sCmn.LBRR_GainIncreases;
            psIndices_LBRR->GainsIndices[ 0 ] = silk_min_int( psIndices_LBRR->GainsIndices[ 0 ], N_LEVELS_QGAIN - 1 );
        }

        /* Decode to get gains in sync with decoder         */
        /* Overwrite unquantized gains with quantized gains */
        silk_gains_dequant( psEncCtrl->Gains_Q16, psIndices_LBRR->GainsIndices,
            &psEnc->sCmn.LBRRprevLastGainIndex, condCoding == CODE_CONDITIONALLY, psEnc->sCmn.nb_subfr );

        /*****************************************/
        /* Noise shaping quantization            */
        /*****************************************/
        if( psEnc->sCmn.nStatesDelayedDecision > 1 || psEnc->sCmn.warping_Q16 > 0 ) {
            silk_NSQ_del_dec( &psEnc->sCmn, &sNSQ_LBRR, psIndices_LBRR, x16,
                psEnc->sCmn.pulses_LBRR[ psEnc->sCmn.nFramesEncoded ], psEncCtrl->PredCoef_Q12[ 0 ], psEncCtrl->LTPCoef_Q14,
                psEncCtrl->AR_Q13, psEncCtrl->HarmShapeGain_Q14, psEncCtrl->Tilt_Q14, psEncCtrl->LF_shp_Q14,
                psEncCtrl->Gains_Q16, psEncCtrl->pitchL, psEncCtrl->Lambda_Q10, psEncCtrl->LTP_scale_Q14, psEnc->sCmn.arch );
        } else {
            silk_NSQ( &psEnc->sCmn, &sNSQ_LBRR, psIndices_LBRR, x16,
                psEnc->sCmn.pulses_LBRR[ psEnc->sCmn.nFramesEncoded ], psEncCtrl->PredCoef_Q12[ 0 ], psEncCtrl->LTPCoef_Q14,
                psEncCtrl->AR_Q13, psEncCtrl->HarmShapeGain_Q14, psEncCtrl->Tilt_Q14, psEncCtrl->LF_shp_Q14,
                psEncCtrl->Gains_Q16, psEncCtrl->pitchL, psEncCtrl->Lambda_Q10, psEncCtrl->LTP_scale_Q14, psEnc->sCmn.arch );
        }

        /* Restore original gains */
        silk_memcpy( psEncCtrl->Gains_Q16, TempGains_Q16, psEnc->sCmn.nb_subfr * sizeof( opus_int32 ) );
    }
}
