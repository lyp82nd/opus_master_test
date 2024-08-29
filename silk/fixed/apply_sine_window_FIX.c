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

/* Apply sine window to signal vector.                                      */
/* Window types:                                                            */
/*    1 -> sine window from 0 to pi/2                                       */
/*    2 -> sine window from pi/2 to pi                                      */
/* Every other sample is linearly interpolated, for speed.                  */
/* Window length must be between 16 and 120 (incl) and a multiple of 4.     */

/* Matlab code for table:
   for k=16:9*4:16+2*9*4, fprintf(' %7.d,', -round(65536*pi ./ (k:4:k+8*4))); fprintf('\n'); end
*/
static const opus_int16 freq_table_Q16[ 27 ] = {
   12111,    9804,    8235,    7100,    6239,    5565,    5022,    4575,    4202,
    3885,    3612,    3375,    3167,    2984,    2820,    2674,    2542,    2422,
    2313,    2214,    2123,    2038,    1961,    1889,    1822,    1760,    1702,
};

void silk_apply_sine_window(
    opus_int16                  px_win[],           /* O    窗函数信号Pointer to windowed signal                                  */
    const opus_int16            px[],               /* I    输入信号Pointer to input signal                                     */
    const opus_int              win_type,           /* I    选择一个窗类型只有1和2可选Selects a window type                                       */
    const opus_int              length              /* I    窗长度一定是4的倍数Window length, multiple of 4                                */
)
{
    opus_int   k, f_Q16, c_Q16;
    opus_int32 S0_Q16, S1_Q16;

    celt_assert( win_type == 1 || win_type == 2 );

    /* 长度必须在16到120之间，并且是4的倍数Length must be in a range from 16 to 120 and a multiple of 4 */
    celt_assert( length >= 16 && length <= 120 );
    celt_assert( ( length & 3 ) == 0 );//直接检测低两位是否为零就可以判断是否是4的倍数

    /* 频率Frequency */
    k = ( length >> 2 ) - 4;//length=16，k=0
    celt_assert( k >= 0 && k <= 26 );
    f_Q16 = (opus_int)freq_table_Q16[ k ];//12111，查表可得0.18479

    /* 用于余弦近似的因子Factor used for cosine approximation */
    c_Q16 = silk_SMULWB( (opus_int32)f_Q16, -f_Q16 );//-2239
    silk_assert( c_Q16 >= -32768 );

    /* 初始化状态initialize state */
    if( win_type == 1 ) {//0-pi/2
        /* 从0开始start from 0 */
        S0_Q16 = 0;
        /* sin（f）的逼近approximation of sin(f) */
        S1_Q16 = f_Q16 + silk_RSHIFT( length, 3 );//12113
    } else {//pi/2-pi
        /* 从1开始start from 1 */
        S0_Q16 = ( (opus_int32)1 << 16 );
        /* cos（f）的逼近approximation of cos(f) */
        S1_Q16 = ( (opus_int32)1 << 16 ) + silk_RSHIFT( c_Q16, 1 ) + silk_RSHIFT( length, 4 );
    }

    /* 使用递归公式：Uses the recursive equation:   sin(n*f) = 2 * cos(f) * sin((n-1)*f) - sin((n-2)*f)    */
    /* 一次处理四个样本4 samples at a time */
    for( k = 0; k < length; k += 4 ) {
        px_win[ k ]     = (opus_int16)silk_SMULWB( silk_RSHIFT( S0_Q16 + S1_Q16, 1 ), px[ k ] );//(((S0_Q16 + S1_Q16) >> 1 )* px[k]) >> 16;
        px_win[ k + 1 ] = (opus_int16)silk_SMULWB( S1_Q16, px[ k + 1] );//(S1_Q16 * px[k + 1]) >> 16;
        S0_Q16 = silk_SMULWB( S1_Q16, c_Q16 ) + silk_LSHIFT( S1_Q16, 1 ) - S0_Q16 + 1;//(S1_Q16 * c_Q16) >> 16 + S1_Q16 << 1 - S0_Q16 + 1;
        S0_Q16 = silk_min( S0_Q16, ( (opus_int32)1 << 16 ) );//取饱和

        px_win[ k + 2 ] = (opus_int16)silk_SMULWB( silk_RSHIFT( S0_Q16 + S1_Q16, 1 ), px[ k + 2] );//(((S0_Q16 + S1_Q16) >> 1 )* px[k+2]) >> 16;
        px_win[ k + 3 ] = (opus_int16)silk_SMULWB( S0_Q16, px[ k + 3 ] );//(S0_Q16 * px[k + 3]) >> 16;
        S1_Q16 = silk_SMULWB( S0_Q16, c_Q16 ) + silk_LSHIFT( S0_Q16, 1 ) - S1_Q16;//(S0_Q16 * c_Q16) >> 16 + S0_Q16 << 1 - S1_Q16 ;
        S1_Q16 = silk_min( S1_Q16, ( (opus_int32)1 << 16 ) );//取饱和
    }
}
