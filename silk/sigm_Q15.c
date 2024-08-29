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

/* Approximate sigmoid function */

#include "SigProc_FIX.h"

/* fprintf(1, '%d, ', round(1024 * ([1 ./ (1 + exp(-(1:5))), 1] - 1 ./ (1 + exp(-(0:5)))))); */
static const opus_int32 sigm_LUT_slope_Q10[ 6 ] = {
    237, 153, 73, 30, 12, 7
};
/* fprintf(1, '%d, ', round(32767 * 1 ./ (1 + exp(-(0:5))))); */
static const opus_int32 sigm_LUT_pos_Q15[ 6 ] = {
    16384, 23955, 28861, 31213, 32178, 32548
};
/* fprintf(1, '%d, ', round(32767 * 1 ./ (1 + exp((0:5))))); */
static const opus_int32 sigm_LUT_neg_Q15[ 6 ] = {
    16384, 8812, 3906, 1554, 589, 219
};
//sigmoid函数映射，具有非线性区间映射的特点
opus_int silk_sigm_Q15(
    opus_int                    in_Q5               /* I                                                                */
)
{
    opus_int ind;

    if( in_Q5 < 0 ) {
        /* 负数的输入Negative input */
        in_Q5 = -in_Q5;//变为正数
        if( in_Q5 >= 6 * 32 ) {//>=192
            return 0;        /* Clip */
        } else {
            /*查表的线性插值 Linear interpolation of look up table */
            ind = silk_RSHIFT( in_Q5, 5 );//除以32变成Q0格式
            return( sigm_LUT_neg_Q15[ ind ] - silk_SMULBB( sigm_LUT_slope_Q10[ ind ], in_Q5 & 0x1F ) );//=sigm_LUT_neg_Q15-sigm_LUT_slope_Q10*in_Q5 & 0x1F 
        }
    } else {
        /* Positive input */
        if( in_Q5 >= 6 * 32 ) {//>192
            return 32767;        /* clip */
        } else {
            /* 查表的线性插值Linear interpolation of look up table */
            ind = silk_RSHIFT( in_Q5, 5 );
            return( sigm_LUT_pos_Q15[ ind ] + silk_SMULBB( sigm_LUT_slope_Q10[ ind ], in_Q5 & 0x1F ) );//=sigm_LUT_neg_Q15+sigm_LUT_slope_Q10*in_Q5 & 0x1F 
        }
    }
}

