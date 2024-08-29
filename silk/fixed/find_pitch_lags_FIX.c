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

#include "main_FIX.h"
#include "stack_alloc.h"
#include "tuning_parameters.h"

/*找到基频,初始的LPC分析 Find pitch lags */
void silk_find_pitch_lags_FIX(
    silk_encoder_state_FIX* psEnc,                                 /* I/O  encoder state                                                               */
    silk_encoder_control_FIX* psEncCtrl,                             /* I/O  encoder control                                                             */
    opus_int16                      res[],                                  /* O    residual                                                                    */
    const opus_int16                x[],                                    /* I    语音信号Speech signal=psEnc->x_buf */
    int                             arch                                    /* I    Run-time architecture                                                       */
)
{//LPC系数计算基于自相关算法，得到自相关auto_corr之后计算反射系数refl_coef，然后根据反射系数递归得到LPC系数A（滤波器系数）然后用这组系数对原始信号进行滤波得到LPC方法估计的信号，后将真实信号x_buf和估计信号的残差（白化）存放在res变量用于计算pitch
    opus_int   buf_len, i, scale;
    opus_int32 thrhld_Q13, res_nrg;
    const opus_int16* x_ptr;//输入信号的首地址=上一个调用中的psEnc->x_buf 
    VARDECL(opus_int16, Wsig);
    opus_int16* Wsig_ptr;
    opus_int32 auto_corr[MAX_FIND_PITCH_LPC_ORDER + 1];
    opus_int16 rc_Q15[MAX_FIND_PITCH_LPC_ORDER];
    opus_int32 A_Q24[MAX_FIND_PITCH_LPC_ORDER];
    opus_int16 A_Q12[MAX_FIND_PITCH_LPC_ORDER];
    SAVE_STACK;
    /******************************************/
    /* 基于采样率建立缓冲长度=音调延迟（2ms）+子帧长度（20ms）+ltp长度（20ms）Set up buffer lengths etc based on Fs  */
    /******************************************/
    buf_len = psEnc->sCmn.la_pitch + psEnc->sCmn.frame_length + psEnc->sCmn.ltp_mem_length;//=336=16+160+160//32+320+320

    /* 安全检查 */
    celt_assert(buf_len >= psEnc->sCmn.pitch_LPC_win_length);

    /*************************************/
    /* 检查lpcAR的系数（无）Estimate LPC AR coefficients      */
    /*************************************/

    /* 计算加窗信号（无）Calculate windowed signal */

    ALLOC(Wsig, psEnc->sCmn.pitch_LPC_win_length, opus_int16);//192（160+32）*opus_int16的数组
    // 为了计算准确性，计算pitch的数据向前和向后各看了2ms，对开始2ms和结束2ms采样点使用正弦窗，中间采样点不需要正弦窗
    /* 第一个LA_LTP要处理的样本First LA_LTP samples */
    x_ptr = x + buf_len - psEnc->sCmn.pitch_LPC_win_length;//x+336-192(control_codec中定义的192)
    Wsig_ptr = Wsig;
    silk_apply_sine_window(Wsig_ptr, x_ptr, 1, psEnc->sCmn.la_pitch);//经历一个正弦窗长度为16，也就是一次处理16(2ms*8khz)个数据，模式1

    /* 中间未加窗样本Middle un - windowed samples */
    Wsig_ptr += psEnc->sCmn.la_pitch;
    x_ptr += psEnc->sCmn.la_pitch;
    silk_memcpy(Wsig_ptr, x_ptr, (psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1)) * sizeof(opus_int16));

    /*最后一个LA_LTP样本 Last LA_LTP samples */
    Wsig_ptr += psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1);
    x_ptr += psEnc->sCmn.pitch_LPC_win_length - silk_LSHIFT(psEnc->sCmn.la_pitch, 1);
    silk_apply_sine_window(Wsig_ptr, x_ptr, 2, psEnc->sCmn.la_pitch);//经历一个正弦窗长度为16，模式2
    /***********************************************************************************/
    /* x                                 第一个窗          未加窗             最后一个窗         +buf_len  */
    /* x_buf                       +144*-- +16 ---*----     +160     ----*----- +16 ------+336      */
    /*音调延迟（16个）+子帧长度（共160拿出后32个采样点，其中前16个作为第一个窗）+ltp长度（共160个采样点，最后的16个最为最后一个窗） */
    /***********************************************************************************/

    /* 计算自相关序列Calculate autocorrelation sequence */
    silk_autocorr(auto_corr, &scale, Wsig, psEnc->sCmn.pitch_LPC_win_length, psEnc->sCmn.pitchEstimationLPCOrder + 1, arch);
    //auto_corr为输出，Wsig加窗的数据，pitch_LPC_win_length=192，pitchEstimationLPCOrder=10
    /*添加白噪声，作为能量的分数 Add white noise, as fraction of energy */
    auto_corr[0] = silk_SMLAWB(auto_corr[0], auto_corr[0], SILK_FIX_CONST(FIND_PITCH_WHITE_NOISE_FRACTION, 16)) + 1;
    //auto_corr[0] + (auto_corr[0] * ((((opus_int32)((1e-3f) * (1 << (16)) + 0.5)))) >> 16);
    //根据反射系数计算的LPC系数是稳定的，使用Schur算法的另一个原因是其递归求解特性会减少计算复杂度
    // 由于命令行参数设置的复杂度是10，因而阶数为16
    // 因而通过反射系数计算LPC系数，而非直接根据自相关计算LPC系数
    /* 根据自相关系数使用schur计算反射系数Calculate the reflection coefficients using schur */
    res_nrg = silk_schur(rc_Q15, auto_corr, psEnc->sCmn.pitchEstimationLPCOrder);

    /* 预测增益Prediction gain */
    psEncCtrl->predGain_Q16 = silk_DIV32_varQ(auto_corr[0], silk_max_int(res_nrg, 1), 16);

    /* 将反射系数转换为预测系数Convert reflection coefficients to prediction coefficients */
    silk_k2a(A_Q24, rc_Q15, psEnc->sCmn.pitchEstimationLPCOrder);

    /*从32位Q24转换为16位Q12系数 Convert From 32 bit Q24 to 16 bit Q12 coefs */
    for (i = 0; i < psEnc->sCmn.pitchEstimationLPCOrder; i++) {
        A_Q12[i] = (opus_int16)silk_SAT16(silk_RSHIFT(A_Q24[i], 12));
    }

    /* Do BWE */
    silk_bwexpander(A_Q12, psEnc->sCmn.pitchEstimationLPCOrder, SILK_FIX_CONST(FIND_PITCH_BANDWIDTH_EXPANSION, 16));

    /*****************************************/
    /* LPC分析滤波LPC analysis filtering                */
    /*****************************************/
    silk_LPC_analysis_filter(res, x, A_Q12, buf_len, psEnc->sCmn.pitchEstimationLPCOrder, psEnc->sCmn.arch);

    if (psEnc->sCmn.indices.signalType != TYPE_NO_VOICE_ACTIVITY && psEnc->sCmn.first_frame_after_reset == 0) {
        /* 基频检测器门限Threshold for pitch estimator */
        thrhld_Q13 = SILK_FIX_CONST(0.6, 13);
        thrhld_Q13 = silk_SMLABB(thrhld_Q13, SILK_FIX_CONST(-0.004, 13), psEnc->sCmn.pitchEstimationLPCOrder);
        thrhld_Q13 = silk_SMLAWB(thrhld_Q13, SILK_FIX_CONST(-0.1, 21), psEnc->sCmn.speech_activity_Q8);
        thrhld_Q13 = silk_SMLABB(thrhld_Q13, SILK_FIX_CONST(-0.15, 13), silk_RSHIFT(psEnc->sCmn.prevSignalType, 1));
        thrhld_Q13 = silk_SMLAWB(thrhld_Q13, SILK_FIX_CONST(-0.1, 14), psEnc->sCmn.input_tilt_Q15);
        thrhld_Q13 = silk_SAT16(thrhld_Q13);

        /*****************************************/
        /* 引用基音估计器Call pitch estimator                  */
        /*****************************************/
        if (silk_pitch_analysis_core(res, psEncCtrl->pitchL, &psEnc->sCmn.indices.lagIndex, &psEnc->sCmn.indices.contourIndex,
            &psEnc->LTPCorr_Q15, psEnc->sCmn.prevLag, psEnc->sCmn.pitchEstimationThreshold_Q16,
            (opus_int)thrhld_Q13, psEnc->sCmn.fs_kHz, psEnc->sCmn.pitchEstimationComplexity, psEnc->sCmn.nb_subfr,
            psEnc->sCmn.arch) == 0)
        {
            psEnc->sCmn.indices.signalType = TYPE_VOICED;
        }
        else {
            psEnc->sCmn.indices.signalType = TYPE_UNVOICED;
        }
    }
    else {
        silk_memset(psEncCtrl->pitchL, 0, sizeof(psEncCtrl->pitchL));
        psEnc->sCmn.indices.lagIndex = 0;
        psEnc->sCmn.indices.contourIndex = 0;
        psEnc->LTPCorr_Q15 = 0;
    }
    RESTORE_STACK;
}
