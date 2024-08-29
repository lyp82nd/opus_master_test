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

/*                                                                      *
 * silk_biquad_alt.c                                              *
 *                                                                      *
 * Second order ARMA filter                                             *
 * Can handle slowly varying filter coefficients                        *
 *                                                                      */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "SigProc_FIX.h"

/* 二阶 ARMA 滤波器Second order ARMA filter, alternative implementation */
void silk_biquad_alt_stride1(
    const opus_int16            *in,                /* I     输入信号input signal                                               */
    const opus_int32            *B_Q28,             /* I     MA参数MA coefficients [3]                                        */
    const opus_int32            *A_Q28,             /* I     ar参数AR coefficients [2]                                        */
    opus_int32                  *S,                 /* I/O   状态向量State vector [2]                                           */
    opus_int16                  *out,               /* O     输出信号output signal                                              */
    const opus_int32            len                 /* I     信号长度（必须是偶数)signal length (must be even)                               */
)
{
    /* 使用2个元素状态向量DIRECT FORM II TRANSPOSED (uses 2 element state vector) */
    opus_int   k;
    opus_int32 inval, A0_U_Q28, A0_L_Q28, A1_U_Q28, A1_L_Q28, out32_Q14;

    /* -A_Q28分成两个部分Negate A_Q28 values and split in two parts */
    A0_L_Q28 = ( -A_Q28[ 0 ] ) & 0x00003FFF;        /* 11417低16位lower part */
    A0_U_Q28 = silk_RSHIFT( -A_Q28[ 0 ], 14 );      /* 31682高16位upper part */
    A1_L_Q28 = ( -A_Q28[ 1 ] ) & 0x00003FFF;        /* 2898低16位lower part */
    A1_U_Q28 = silk_RSHIFT( -A_Q28[ 1 ], 14 );      /* -15336高16位upper part */

    for( k = 0; k < len; k++ ) {
        /* S[ 0 ], S[ 1 ]: Q12 */
        inval = in[ k ];
        out32_Q14 = silk_LSHIFT( silk_SMLAWB( S[ 0 ], B_Q28[ 0 ], inval ), 2 );
        //((S[0]) + (B_Q28[0] * inval) >> 16) << 2;
        int a = silk_SMULWB(out32_Q14, A0_L_Q28);
        //(out32_Q14 * A0_L_Q28) >> 16;
        int b = silk_RSHIFT_ROUND(a, 14);
         ((a >> 13) + 1) >> 1;
        S[ 0 ] = S[1] + b;
        S[ 0 ] = silk_SMLAWB( S[ 0 ], out32_Q14, A0_U_Q28 );
        //S[0] + (out32_Q14 * A0_U_Q28) >> 16;
        S[ 0 ] = silk_SMLAWB( S[ 0 ], B_Q28[ 1 ], inval);
        //(S[0]) + (B_Q28[1] * inval) >> 16;

        S[ 1 ] = silk_RSHIFT_ROUND( silk_SMULWB( out32_Q14, A1_L_Q28 ), 14 );
        ((((out32_Q14 * A1_L_Q28) >> 16) >> ((14) - 1)) + 1) >> 1;
        S[ 1 ] = silk_SMLAWB( S[ 1 ], out32_Q14, A1_U_Q28 );
        S[ 1 ] = silk_SMLAWB( S[ 1 ], B_Q28[ 2 ], inval );

        /*回到 Q0并饱和处理 Scale back to Q0 and saturate */
        out[ k ] = (opus_int16)silk_SAT16( silk_RSHIFT( out32_Q14 + (1<<14) - 1, 14 ) );
    }
}

void silk_biquad_alt_stride2_c(
    const opus_int16            *in,                /* I     input signal                                               */
    const opus_int32            *B_Q28,             /* I     MA coefficients [3]                                        */
    const opus_int32            *A_Q28,             /* I     AR coefficients [2]                                        */
    opus_int32                  *S,                 /* I/O   State vector [4]                                           */
    opus_int16                  *out,               /* O     output signal                                              */
    const opus_int32            len                 /* I     signal length (must be even)                               */
)
{
    /* DIRECT FORM II TRANSPOSED (uses 2 element state vector) */
    opus_int   k;
    opus_int32 A0_U_Q28, A0_L_Q28, A1_U_Q28, A1_L_Q28, out32_Q14[ 2 ];

    /* Negate A_Q28 values and split in two parts */
    A0_L_Q28 = ( -A_Q28[ 0 ] ) & 0x00003FFF;        /* lower part */
    A0_U_Q28 = silk_RSHIFT( -A_Q28[ 0 ], 14 );      /* upper part */
    A1_L_Q28 = ( -A_Q28[ 1 ] ) & 0x00003FFF;        /* lower part */
    A1_U_Q28 = silk_RSHIFT( -A_Q28[ 1 ], 14 );      /* upper part */

    for( k = 0; k < len; k++ ) {
        /* S[ 0 ], S[ 1 ], S[ 2 ], S[ 3 ]: Q12 */
        out32_Q14[ 0 ] = silk_LSHIFT( silk_SMLAWB( S[ 0 ], B_Q28[ 0 ], in[ 2 * k + 0 ] ), 2 );
        out32_Q14[ 1 ] = silk_LSHIFT( silk_SMLAWB( S[ 2 ], B_Q28[ 0 ], in[ 2 * k + 1 ] ), 2 );

        S[ 0 ] = S[ 1 ] + silk_RSHIFT_ROUND( silk_SMULWB( out32_Q14[ 0 ], A0_L_Q28 ), 14 );
        S[ 2 ] = S[ 3 ] + silk_RSHIFT_ROUND( silk_SMULWB( out32_Q14[ 1 ], A0_L_Q28 ), 14 );
        S[ 0 ] = silk_SMLAWB( S[ 0 ], out32_Q14[ 0 ], A0_U_Q28 );
        S[ 2 ] = silk_SMLAWB( S[ 2 ], out32_Q14[ 1 ], A0_U_Q28 );
        S[ 0 ] = silk_SMLAWB( S[ 0 ], B_Q28[ 1 ], in[ 2 * k + 0 ] );
        S[ 2 ] = silk_SMLAWB( S[ 2 ], B_Q28[ 1 ], in[ 2 * k + 1 ] );

        S[ 1 ] = silk_RSHIFT_ROUND( silk_SMULWB( out32_Q14[ 0 ], A1_L_Q28 ), 14 );
        S[ 3 ] = silk_RSHIFT_ROUND( silk_SMULWB( out32_Q14[ 1 ], A1_L_Q28 ), 14 );
        S[ 1 ] = silk_SMLAWB( S[ 1 ], out32_Q14[ 0 ], A1_U_Q28 );
        S[ 3 ] = silk_SMLAWB( S[ 3 ], out32_Q14[ 1 ], A1_U_Q28 );
        S[ 1 ] = silk_SMLAWB( S[ 1 ], B_Q28[ 2 ], in[ 2 * k + 0 ] );
        S[ 3 ] = silk_SMLAWB( S[ 3 ], B_Q28[ 2 ], in[ 2 * k + 1 ] );

        /* Scale back to Q0 and saturate */
        out[ 2 * k + 0 ] = (opus_int16)silk_SAT16( silk_RSHIFT( out32_Q14[ 0 ] + (1<<14) - 1, 14 ) );
        out[ 2 * k + 1 ] = (opus_int16)silk_SAT16( silk_RSHIFT( out32_Q14[ 1 ] + (1<<14) - 1, 14 ) );
    }
}
