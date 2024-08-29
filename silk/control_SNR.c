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

#include "main.h"
#include "tuning_parameters.h"

/* 重对偶量化器的信噪比控制Control SNR of redidual quantizer */
opus_int silk_control_SNR(
    silk_encoder_state          *psEncC,                        /* I/O  Pointer to Silk encoder state               */
    opus_int32                  TargetRate_bps                  /* I    Target max bitrate (bps)                    */
)
{//将输入的比特率转化成信噪比，方式是通过查表线性映射
    opus_int k, ret = SILK_NO_ERROR;
    opus_int32 frac_Q6;
    const opus_int32 *rateTable;

    /* 设置比特率/编码质量Set bitrate/coding quality */
    TargetRate_bps = silk_LIMIT( TargetRate_bps, MIN_TARGET_RATE_BPS, MAX_TARGET_RATE_BPS );
    if( TargetRate_bps != psEncC->TargetRate_bps ) {
        psEncC->TargetRate_bps = TargetRate_bps;

        /* 如果是新的TargetRate_bps，则转换为SNR_dB值If new TargetRate_bps, translate to SNR_dB value */
        if( psEncC->fs_kHz == 8 ) {
            rateTable = silk_TargetRate_table_NB;//[8000,9400,11500,13500,17500,25000,80000]
        } else if( psEncC->fs_kHz == 12 ) {
            rateTable = silk_TargetRate_table_MB;
        } else {
            rateTable = silk_TargetRate_table_WB;
        }

        /* 在这些计算中降低10ms模式的比特率Reduce bitrate for 10 ms modes in these calculations */
        if( psEncC->nb_subfr == 2 ) {
            TargetRate_bps -= REDUCE_BITRATE_10_MS_BPS;
        }

        /*在表中查找比特率间隔并进行插值 Find bitrate interval in table and interpolate */
        for( k = 1; k < TARGET_RATE_TAB_SZ; k++ ) {
            if( TargetRate_bps <= rateTable[ k ] ) {
                frac_Q6 = silk_DIV32( silk_LSHIFT( TargetRate_bps - rateTable[ k - 1 ], 6 ), rateTable[ k ] - rateTable[ k - 1 ] );//=16   ((TargetRate_bps - rateTable[k - 1]) / (rateTable[k] - rateTable[k - 1]))>>6;
                psEncC->SNR_dB_Q7 = silk_LSHIFT( silk_SNR_table_Q1[ k - 1 ], 6 ) + silk_MUL( frac_Q6, silk_SNR_table_Q1[ k ] - silk_SNR_table_Q1[ k - 1 ] );//=2000     frac_Q6 * (silk_SNR_table_Q1[k] - silk_SNR_table_Q1[k - 1]);
                //silk_SNR_table_Q1[ TARGET_RATE_TAB_SZ ] = {18, 29, 38, 40, 46, 52, 62, 84};
                break;
            }
        }
    }

    return ret;
}
