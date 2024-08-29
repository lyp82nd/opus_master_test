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

#include "SigProc_FIX.h"
#include "resampler_private.h"

/* 向上采样2倍，高质量Upsample by a factor 2, high quality */
/* 对2x上采样使用二阶全通滤波器，然后是Nyquist上方的陷波滤波器Uses 2nd order allpass filters for the 2x upsampling, followed by a      */
/* notch filter just above Nyquist.                                         */
void silk_resampler_private_up2_HQ(
    opus_int32                      *S,             /* I/O  Resampler state [ 6 ]       */
    opus_int16                      *out,           /* O    Output signal [ 2 * len ]   */
    const opus_int16                *in,            /* I    Input signal [ len ]        */
    opus_int32                      len             /* I    Number of input samples     */
)
{
    opus_int32 k;
    opus_int32 in32, out32_1, out32_2, Y, X;

    silk_assert( silk_resampler_up2_hq_0[ 0 ] > 0 );
    silk_assert( silk_resampler_up2_hq_0[ 1 ] > 0 );
    silk_assert( silk_resampler_up2_hq_0[ 2 ] < 0 );
    silk_assert( silk_resampler_up2_hq_1[ 0 ] > 0 );
    silk_assert( silk_resampler_up2_hq_1[ 1 ] > 0 );
    silk_assert( silk_resampler_up2_hq_1[ 2 ] < 0 );

    /* 数据是Q10表示  Internal variables and state are in Q10 format */
    for( k = 0; k < len; k++ ) {
        /* Convert to Q10 */
        in32 = silk_LSHIFT( (opus_int32)in[ k ], 10 );

        /* 偶数输出样本的第一个全通区  First all-pass section for even output sample */
        Y       = silk_SUB32( in32, S[ 0 ] );//in32-S[0];
        X       = silk_SMULWB( Y, silk_resampler_up2_hq_0[ 0 ] );//(Y * silk_resampler_up2_hq_0[0]) >> 16;
        out32_1 = silk_ADD32( S[ 0 ], X );//S[0] + X
        S[ 0 ]  = silk_ADD32( in32, X );//in32+X

        /* 偶数输出样本的第二个全通区Second all-pass section for even output sample */
        Y       = silk_SUB32( out32_1, S[ 1 ] );
        X       = silk_SMULWB( Y, silk_resampler_up2_hq_0[ 1 ] );
        out32_2 = silk_ADD32( S[ 1 ], X );
        S[ 1 ]  = silk_ADD32( out32_1, X );

        /*偶数输出样本的第三个全通区Third all-pass section for even output sample */
        Y       = silk_SUB32( out32_2, S[ 2 ] );
        X       = silk_SMLAWB( Y, Y, silk_resampler_up2_hq_0[ 2 ] );
        out32_1 = silk_ADD32( S[ 2 ], X );
        S[ 2 ]  = silk_ADD32( out32_2, X );

        /*在Q15中应用增益，转换回int16并存储到输出 Apply gain in Q15, convert back to int16 and store to output */
        out[ 2 * k ] = (opus_int16)silk_SAT16( silk_RSHIFT_ROUND( out32_1, 10 ) );

        /* 奇数输出样本的第一个全通区First all-pass section for odd output sample */
        Y       = silk_SUB32( in32, S[ 3 ] );
        X       = silk_SMULWB( Y, silk_resampler_up2_hq_1[ 0 ] );
        out32_1 = silk_ADD32( S[ 3 ], X );
        S[ 3 ]  = silk_ADD32( in32, X );

        /*奇数输出样本的第二个全通区 Second all-pass section for odd output sample */
        Y       = silk_SUB32( out32_1, S[ 4 ] );
        X       = silk_SMULWB( Y, silk_resampler_up2_hq_1[ 1 ] );
        out32_2 = silk_ADD32( S[ 4 ], X );
        S[ 4 ]  = silk_ADD32( out32_1, X );

        /* 奇数输出样本的第三个全通区Third all-pass section for odd output sample */
        Y       = silk_SUB32( out32_2, S[ 5 ] );
        X       = silk_SMLAWB( Y, Y, silk_resampler_up2_hq_1[ 2 ] );
        out32_1 = silk_ADD32( S[ 5 ], X );
        S[ 5 ]  = silk_ADD32( out32_2, X );

        /*在Q15中应用增益，转换回int16并存储到输出 Apply gain in Q15, convert back to int16 and store to output */
        out[ 2 * k + 1 ] = (opus_int16)silk_SAT16( silk_RSHIFT_ROUND( out32_1, 10 ) );
    }
}

void silk_resampler_private_up2_HQ_wrapper(
    void                            *SS,            /* I/O  Resampler state (unused)    */
    opus_int16                      *out,           /* O    Output signal [ 2 * len ]   */
    const opus_int16                *in,            /* I    Input signal [ len ]        */
    opus_int32                      len             /* I    Number of input samples     */
)
{
    silk_resampler_state_struct *S = (silk_resampler_state_struct *)SS;
    silk_resampler_private_up2_HQ( S->sIIR, out, in, len );
}
