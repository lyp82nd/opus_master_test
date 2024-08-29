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
#ifdef FIXED_POINT
#include "fixed/main_FIX.h"
#else
#include "float/main_FLP.h"
#endif
#include "tuning_parameters.h"
#include "cpu_support.h"

/*********************************/
/* Initialize Silk Encoder state */
/*********************************/
opus_int silk_init_encoder(
    silk_encoder_state_Fxx          *psEnc,                                 /* I/O  Pointer to Silk FIX encoder state       传入&psEnc->state_Fxx[ n ]                                    */
    int                              arch                                   /* I    Run-time architecture                                                       */
)
{
    opus_int ret = 0;

    /* Clear the entire encoder state */
    silk_memset( psEnc, 0, sizeof( silk_encoder_state_Fxx ) );//先将8648个字节置零
    
    psEnc->sCmn.arch = arch;//将架构传给silk
                            //初始化高通滤波器状态
    psEnc->sCmn.variable_HP_smth1_Q15 = silk_LSHIFT( silk_lin2log( SILK_FIX_CONST( VARIABLE_HP_MIN_CUTOFF_HZ, 16 ) ) - ( 16 << 7 ), 8 );//将可变高通滤波器的最小截止频率转换为Q15格式的定点数，并将其存储在 psEnc->sCmn.variable_HP_smth1_Q15。
    psEnc->sCmn.variable_HP_smth2_Q15 = psEnc->sCmn.variable_HP_smth1_Q15;//这两个参数决定了每一帧高通滤波的截止频率，同时截止频率还和上一帧语音是否为浊音有关，是浊音则需要重新计算截止频率，不是浊音则不需要重新计算

    /* 禁用LSF插值和音高预测的功能Used to deactivate LSF interpolation, pitch prediction */
    psEnc->sCmn.first_frame_after_reset = 1;//停止NLSF插值和基频预测（不是基频检测）

    /* Initialize Silk VAD */              //初始化VAD结构体
    ret += silk_VAD_Init( &psEnc->sCmn.sVAD );

    return  ret;
}
