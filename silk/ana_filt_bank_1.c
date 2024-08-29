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

/* ����һ��ȫͨ�˲�����2Ƶ���˲������ϵ��=10788Coefficients for 2-band filter bank based on first-order allpass filters */
static opus_int16 A_fb1_20 = 5394 << 1;
/* ����һ��ȫͨ�˲�����2Ƶ���˲������ϵ��=-24290��(opus_int16)(20623 << 1) */
static opus_int16 A_fb1_21 = -24290; 

/* ʹ��һ��ȫͨ�˲������źŷָ�Ϊ������ȡƵ����ͨ������һ��ȫͨ�˲���ʵ�ֶ����˲��ṹ��silk_ana_filt_bank_1������ʵ�ֳ�ȡ�²�����Split signal into two decimated bands using first-order allpass filters */
void silk_ana_filt_bank_1(
    const opus_int16            *in,                /* I    Input signal [N]                                            */
    opus_int32                  *S,                 /* I/O  ״̬����������λ�ã�State vector [2]                                            */
    opus_int16                  *outL,              /* O    Low band [N/2]                                              */
    opus_int16                  *outH,              /* O    High band [N/2]                                             */
    const opus_int32            N                   /* I    Number of input samples=160                                 */
)
{
    opus_int      k, N2 = silk_RSHIFT( N, 1 );
    opus_int32    in32, X, Y, out_1, out_2;

    /* �ڲ�������״̬����Q10��ʽInternal variables and state are in Q10 format */
    for( k = 0; k < N2; k++ ) {
        /* ת����Q10��ʽConvert to Q10 */
        in32 = silk_LSHIFT( (opus_int32)in[ 2 * k ], 10 );//��Ƶ�����������Q10
        /* ż������������ȫͨ��All-pass section for even input sample */
        Y      = silk_SUB32( in32, S[ 0 ] );//in32-S[0],S[ 0 ]����һ��ż�����ݾ���,��ʼ��Ϊ0
        X = silk_SMLAWB(Y, Y, A_fb1_21);//in32-S[0]+((in32-S[0]) * A_fb1_21) >> 16
        out_1  = silk_ADD32( S[ 0 ], X );// in32+((in32-S[0]) * A_fb1_21) >> 16
        S[ 0 ] = silk_ADD32( in32, X );//in32+in32-S[0]+((in32-S[0]) * A_fb1_21) >> 16

        /* ת����Q10Convert to Q10 */
        in32 = silk_LSHIFT( (opus_int32)in[ 2 * k + 1 ], 10 );
        /* ��������������ȫͨ�Σ�����ӵ�ǰһ���ֵ����All-pass section for odd input sample, and add to output of previous section */
        Y = silk_SUB32(in32, S[1]); //in32-S[1](����һ���������ݾ���)
        X = silk_SMULWB(Y, A_fb1_20);//((in32-S[1])* A_fb1_20) >> 16
        out_2  = silk_ADD32( S[ 1 ], X );//S[1] + ((in32-S[1])* A_fb1_20) >> 16
        S[ 1 ] = silk_ADD32( in32, X );//in32+((in32-S[1])* A_fb1_20) >> 16

        int a = silk_ADD32(out_2, out_1);
        int b = silk_RSHIFT_ROUND(a, 11);
        ((11) == 1 ? ((a) >> 1) + ((a) & 1) : ((a >> 10) + 1) >> 1);
        /* �ӷ�/������ת����int16���洢�����Add/subtract, convert back to int16 and store to output */
        outL[ k ] = (opus_int16)silk_SAT16( b );//��Ƶ�Ӵ�������һ�εĻ���ֱ��1k
        outH[ k ] = (opus_int16)silk_SAT16( silk_RSHIFT_ROUND( silk_SUB32( out_2, out_1 ), 11 ) );//��Ƶ�Ӵ�ֱ�����
    }
}
