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
#include "stack_alloc.h"

/* Silk VAD noise level estimation */
# if !defined(OPUS_X86_MAY_HAVE_SSE4_1)
static OPUS_INLINE void silk_VAD_GetNoiseLevels(
    const opus_int32             pX[ VAD_N_BANDS ], /* I    subband energies                            */
    silk_VAD_state              *psSilk_VAD         /* I/O  Pointer to Silk VAD state                   */
);
#endif

/**********************************/
/* Initialization of the Silk VAD */
/**********************************/
opus_int silk_VAD_Init(                                         /* O    Return value, 0 if success                  */
    silk_VAD_state              *psSilk_VAD                     /* I/O  Pointer to Silk VAD state               传入&psEnc->sCmn.sVAD    */
)
{//VAD初始化函数初始化了噪声的基值也就是默认背景噪声是粉红噪声（以噪声等级偏移变量存在），同时也初始化了噪声等级的倒数，初始化了帧计数器初值设置为15，在计算平滑噪声的时候需要用到帧计数器记录VAD处理的帧数来计算平滑噪声的参数，保证前1000帧（20s）参数最大，使噪声平滑的很快，初始化了每个子带的能量噪声比值默认20db也就是信号能量是噪声的100倍
    opus_int b, ret = 0;

    /* reset state memory */
    silk_memset( psSilk_VAD, 0, sizeof( silk_VAD_state ) );//将结构体置零

    /* init noise levels */
    /* Initialize array with approx pink noise levels (psd proportional to inverse of frequency) */
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NoiseLevelBias[ b ] = silk_max_32( silk_DIV32_16( VAD_NOISE_LEVELS_BIAS, b + 1 ), 1 );//粉红噪声的偏移数值【50，25，16，12】
    }

    /* Initialize state */
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NL[ b ]     = silk_MUL( 100, psSilk_VAD->NoiseLevelBias[ b ] );//基础噪声设置为粉红噪声背景
        psSilk_VAD->inv_NL[ b ] = silk_DIV32( silk_int32_MAX, psSilk_VAD->NL[ b ] );//反转噪声
    }
    psSilk_VAD->counter = 15;                                                       //帧计数器                  用来统计VAD的帧数，20ms为一帧，在开始设置为15，后面需要根据帧数来计算噪声平滑系数，前20s也就是前1000帧需要很快的平滑噪声

    /* 初始化子带平滑能量噪声比 init smoothed energy-to-noise ratio*/
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NrgRatioSmth_Q8[ b ] = 100 * 256;       /* 100 * 256 --> 20 dB SNR */     //每个频带中的能量噪声比25600->20dbSNR，后面用于计算每个频带的信噪比
    }

    return( ret );
}

/* Weighting factors for tilt measure */
static const opus_int32 tiltWeights[ VAD_N_BANDS ] = { 30000, 6000, -12000, -12000 };

/***************************************/
/* Get the speech activity level in Q8 */
/***************************************/
opus_int silk_VAD_GetSA_Q8_c(                                   /* O    Return value, 0 if success                  */
    silk_encoder_state          *psEncC,                        /* I/O  Encoder state                               */
    const opus_int16            pIn[]                           /* I    PCM input                                   */
)
{
    opus_int   SA_Q15, pSNR_dB_Q7, input_tilt;
    opus_int   decimated_framelength1, decimated_framelength2;
    opus_int   decimated_framelength;
    opus_int   dec_subframe_length, dec_subframe_offset, SNR_Q7, i, b, s;
    opus_int32 sumSquared, smooth_coef_Q16;
    opus_int16 HPstateTmp;
    VARDECL( opus_int16, X );
    opus_int32 Xnrg[ VAD_N_BANDS ];//记录了每个子带的能量
    opus_int32 NrgToNoiseRatio_Q8[ VAD_N_BANDS ];
    opus_int32 speech_nrg, x_tmp;
    opus_int   X_offset[ VAD_N_BANDS ];
    opus_int   ret = 0;
    silk_VAD_state *psSilk_VAD = &psEncC->sVAD;
    SAVE_STACK;

    /* Safety checks */
    silk_assert( VAD_N_BANDS == 4 );
    celt_assert( MAX_FRAME_LENGTH >= psEncC->frame_length );
    celt_assert( psEncC->frame_length <= 512 );
    celt_assert( psEncC->frame_length == 8 * silk_RSHIFT( psEncC->frame_length, 3 ) );

    /***********************/
    /* 滤波和分样Filter and Decimate */
    /***********************/
    decimated_framelength1 = silk_RSHIFT( psEncC->frame_length, 1 );//80，frame_length=160，二分之帧
    decimated_framelength2 = silk_RSHIFT( psEncC->frame_length, 2 );//40，四分之帧
    decimated_framelength = silk_RSHIFT( psEncC->frame_length, 3 );//20，八分之帧
    /* Decimate into 4 bands:
       0       L      3L       L              3L                             5L
               -      --       -              --                             --
               8       8       2               4                              4

       [0-1 kHz| temp. |1-2 kHz|    2-4 kHz    |            4-8 kHz           |

       它们被安排为在下采样过程中允许最小的（frame_length/4）额外划分的空间They're arranged to allow the minimal ( frame_length / 4 ) extra
       scratch space during the downsampling process */
    X_offset[ 0 ] = 0;//decimated_framelength=20
    X_offset[ 1 ] = decimated_framelength + decimated_framelength2;//60
    X_offset[ 2 ] = X_offset[ 1 ] + decimated_framelength;//80
    X_offset[ 3 ] = X_offset[ 2 ] + decimated_framelength2;/*X_offset=0，60，80，120,数组变量X_offset分别记录了0-1kHz，1-2kHz，2-4kHz，4-8kHz四个子带时域点的起止偏移位置*/
    ALLOC( X, X_offset[ 3 ] + decimated_framelength1, opus_int16 );//后面计算各个子带能量时会用到
    //silk_ana_filt_bank_1函数是实现抽取下采样，输出的数据是Q8类型
    /* 0-8 kHz to 0-4 kHz and 4-8 kHz */
    silk_ana_filt_bank_1( pIn, &psSilk_VAD->AnaState[  0 ],
        X, &X[ X_offset[ 3 ] ], psEncC->frame_length );

    /* 0-4 kHz to 0-2 kHz and 2-4 kHz */
    silk_ana_filt_bank_1( X, &psSilk_VAD->AnaState1[ 0 ],
        X, &X[ X_offset[ 2 ] ], decimated_framelength1 );

    /* 0-2 kHz to 0-1 kHz and 1-2 kHz */
    silk_ana_filt_bank_1( X, &psSilk_VAD->AnaState2[ 0 ],
        X, &X[ X_offset[ 1 ] ], decimated_framelength2 );

    //decimated_framelength的长度是40，这里对频率最低子带使用高通滤波，
//这是因为人的发声一般在几十赫兹以上，所以对低频信号进行适当削弱，减弱的方式是减去前一个数据
    /*********************************************/
    /* 最低频带上的HP滤波器（微分器）HP filter on lowest band (differentiator) */
    /*********************************************/
    X[ decimated_framelength - 1 ] = silk_RSHIFT( X[ decimated_framelength - 1 ], 1 );//最低子带的最后一数据右移一位
    HPstateTmp = X[ decimated_framelength - 1 ];//高通滤波器状态
    for( i = decimated_framelength - 1; i > 0; i-- ) {
        X[ i - 1 ]  = silk_RSHIFT( X[ i - 1 ], 1 );
        X[ i ]     -= X[ i - 1 ];
    }
    X[ 0 ] -= psSilk_VAD->HPstate;
    psSilk_VAD->HPstate = HPstateTmp;//高通滤波器状态

    /*************************************/
    /* 计算每个子带的能量Calculate the energy in each band */
    /*************************************/
    for( b = 0; b < VAD_N_BANDS; b++ ) //遍历四个子带
    {
        /* 根据所处的子带位置，计算该子带时域点数，子带0就是20，在非均匀划分的频带中查找抽取的帧长度Find the decimated framelength in the non-uniformly divided bands */
        decimated_framelength = silk_RSHIFT( psEncC->frame_length, silk_min_int( VAD_N_BANDS - b, VAD_N_BANDS - 1 ) );//当前子带的长度，silk_min_int( VAD_N_BANDS - b, VAD_N_BANDS - 1 )只是确保不会出错

        /* 计算子带的时域点个数，将每一个子带再分成四份子帧Split length into subframe lengths */
        dec_subframe_length = silk_RSHIFT( decimated_framelength, VAD_INTERNAL_SUBFRAMES_LOG2 );//b=0,=5
        dec_subframe_offset = 0;

        /* 计算每个子帧的能量Compute energy per sub-frame */
        /* 各子带的能量初始值为前一个子带的能量值，用最后一个子帧的能量总和初始化initialize with summed energy of last subframe */
        Xnrg[ b ] = psSilk_VAD->XnrgSubfr[ b ];//0,
        for( s = 0; s < VAD_INTERNAL_SUBFRAMES; s++ ) //遍历每个子带的四个子帧
        {
            sumSquared = 0;                                                          //记录的是分成四份数据后每一份数据能量
            //前三帧的能量都是每个采样点的能量求和载相加到一起，最后一帧是能量是每个采样点求能量和后右移一位sumSquared >> 1，再加到一起
            for( i = 0; i < dec_subframe_length; i++ ) //遍历每个子帧中的每个数据
            {
                /* 能量将小于dec_subframe_length*（silk_int16_MIN/8）^2。The energy will be less than dec_subframe_length * ( silk_int16_MIN / 8 ) ^ 2.            */
                /* 因此，我们可以在没有溢出风险的情况下进行累积（除非dec_subframe_length>128）Therefore we can accumulate with no risk of overflow (unless dec_subframe_length > 128)  */
                x_tmp = silk_RSHIFT(X[ X_offset[ b ] + i + dec_subframe_offset ], 3 );//输入音频数据右移三位相当于/8
                sumSquared = silk_SMLABB( sumSquared, x_tmp, x_tmp );//(sumSquared+x_tmp * x_tmp/64)

                /* Safety check */
                silk_assert( sumSquared >= 0 );
            }

            /*Add/saturate summed energy of current subframe */
            if( s < VAD_INTERNAL_SUBFRAMES - 1 ) {
                Xnrg[b] = silk_ADD_POS_SAT32(Xnrg[b], sumSquared);//(Xnrg[b] + sumSquared)                                       //将子带能量记录到Xnrg中
            } else {
                /* Look-ahead subframe */
                Xnrg[ b ] = silk_ADD_POS_SAT32( Xnrg[ b ], silk_RSHIFT( sumSquared, 1 ) );//Xnrg[b] + sumSquared >> 1            //最后一份数据/2再加入到当前子带能量中去
            }

            dec_subframe_offset += dec_subframe_length;//数据下标移动
        }
        psSilk_VAD->XnrgSubfr[ b ] = sumSquared;                                                            //将子带最后一份数据的能量作为下一帧（不是下一子带）的初始值
    }

    /********************/
    /* 噪声检测Noise estimation */
    /********************/
    silk_VAD_GetNoiseLevels( &Xnrg[ 0 ], psSilk_VAD );

    /***********************************************/
    /* 信噪比SNR Signal-plus-noise to noise ratio estimation */
    /***********************************************/
    sumSquared = 0;
    input_tilt = 0;
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        speech_nrg = Xnrg[ b ] - psSilk_VAD->NL[ b ];//纯语音信号的能量=信号能量-噪声等级，用来检测是否存在信噪比
        if( speech_nrg > 0 ) {
            /* Divide, with sufficient resolution */
            if( ( Xnrg[ b ] & 0xFF800000 ) == 0 ) //高九位没有值
            {
                NrgToNoiseRatio_Q8[ b ] = silk_DIV32( silk_LSHIFT( Xnrg[ b ], 8 ), psSilk_VAD->NL[ b ] + 1 );//重新考虑数值的定点位置，并将噪声等级加一取整，之后两者相除
            } 
            else {
                NrgToNoiseRatio_Q8[ b ] = silk_DIV32( Xnrg[ b ], silk_RSHIFT( psSilk_VAD->NL[ b ], 8 ) + 1 );
            }
            //重新考虑精度，并计算snr
            /* 转换到对数域Convert to log domain */
            SNR_Q7 = silk_lin2log(NrgToNoiseRatio_Q8[b]) - 8 * 128;//128log(Xb)-8*128=2^7 *log(Xb/2^8)，内部将Q8格式消除，乘上128得到Q7格式的数

            /* 平方和Sum-of-squares */
            sumSquared = silk_SMLABB( sumSquared, SNR_Q7, SNR_Q7 );          /* Q14 */

            /* 倾斜测量Tilt measure */
            if( speech_nrg < ( (opus_int32)1 << 20 ) ) {
                /* 小型子带语音能量的缩减SNR值Scale down SNR value for small subband speech energies */
                SNR_Q7 = silk_SMULWB( silk_LSHIFT( silk_SQRT_APPROX( speech_nrg ), 6 ), SNR_Q7 );
            }
            input_tilt = silk_SMLAWB(input_tilt, tiltWeights[b], SNR_Q7);//谱倾斜计算，input_tilt+所有子带的(tiltWeights*SNR)
        } else {
            NrgToNoiseRatio_Q8[ b ] = 256;
        }
    }

    /* 平方的平均值Mean-of-squares */
    sumSquared = silk_DIV32_16( sumSquared, VAD_N_BANDS ); /* Q14 */

    /* 均方根近似，缩放到dBs，并写入输出指针Root-mean-square approximation, scale to dBs, and write to output pointer */
    pSNR_dB_Q7 = (opus_int16)( 3 * silk_SQRT_APPROX( sumSquared ) ); /* Q7 */

    /*********************************/
    /* 语音概率估计Speech Probability Estimation */
    /*********************************/
    SA_Q15 = silk_sigm_Q15(silk_SMULWB(VAD_SNR_FACTOR_Q16, pSNR_dB_Q7) - VAD_NEGATIVE_OFFSET_Q5);//((45000*pSNR_dB_Q7)>>16  -128=-128)结果=589

    /**************************/
    /*频率倾斜测量 Frequency Tilt Measure */
    /**************************/
    psEncC->input_tilt_Q15 = silk_LSHIFT( silk_sigm_Q15( input_tilt ) - 16384, 1 );

    /**************************************************/
    /* 根据语音能量对语音概率进行缩放Scale the sigmoid output based on power levels */
    /**************************************************/
    speech_nrg = 0;
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        /* 积累没有噪声能量的信号，更高的频带具有更大的权重Accumulate signal-without-noise energies, higher frequency bands have more weight */
        speech_nrg += ( b + 1 ) * silk_RSHIFT( Xnrg[ b ] - psSilk_VAD->NL[ b ], 4 );//由于高频子带能量小，故这里进行了（b+1）加权，因为是4个子带，所以每个子带的语音能量除以4以获得四个字带的平均值
    }

    if( psEncC->frame_length == 20 * psEncC->fs_kHz ) {//对20ms帧的情况，除以2以得到10ms帧计算的能量，这里以右移一位实现除2的运算
        speech_nrg = silk_RSHIFT32( speech_nrg, 1 );
    }
    /* //根据前面计算的能量，调整语音概率Power scaling */
    if( speech_nrg <= 0 ) {//如果小于等于零，说明前面在加权计算能量的时候溢出了，这时缩小SA_Q15的值
        SA_Q15 = silk_RSHIFT( SA_Q15, 1 );
    } else if( speech_nrg < 16384 ) {
        speech_nrg = silk_LSHIFT32( speech_nrg, 16 );
        //均方根能量之后转换到Q15表示（32768=2^15 ）
        /* square-root */
        speech_nrg = silk_SQRT_APPROX( speech_nrg );
        SA_Q15 = silk_SMULWB( 32768 + speech_nrg, SA_Q15 );
        //((((32768 + speech_nrg)* ( SA_Q15)) >> 16));////////////////////////////////////////////////
    }

    /* 记录数据，格式转换为Q8Copy the resulting speech activity in Q8 */
    psEncC->speech_activity_Q8 = silk_min_int( silk_RSHIFT( SA_Q15, 7 ), silk_uint8_MAX );

    /***********************************/
    /* 能量级和信噪比估计Energy Level and SNR estimation */
    /***********************************/
    /* 平滑参数，有语音概率计算而得Smoothing coefficient */
    smooth_coef_Q16 = silk_SMULWB( VAD_SNR_SMOOTH_COEF_Q18, silk_SMULWB( (opus_int32)SA_Q15, SA_Q15 ) );//(4096*(SA_Q15* SA_Q15) >> 16) >> 16;

    if( psEncC->frame_length == 10 * psEncC->fs_kHz ) {
        smooth_coef_Q16 >>= 1;
    }//如果是10ms则平滑系数减少一半

    for( b = 0; b < VAD_N_BANDS; b++ ) {
        /* 计算每个子带平滑能量噪声比compute smoothed energy-to-noise ratio per band */
        psSilk_VAD->NrgRatioSmth_Q8[ b ] = silk_SMLAWB( psSilk_VAD->NrgRatioSmth_Q8[ b ],NrgToNoiseRatio_Q8[ b ] - psSilk_VAD->NrgRatioSmth_Q8[ b ], smooth_coef_Q16 );
                                                     //psSilk_VAD->NrgRatioSmth_Q8[b] + ((NrgToNoiseRatio_Q8[b] - psSilk_VAD->NrgRatioSmth_Q8[b]) *smooth_coef_Q16) >> 16;
        /* 每个频带的信噪比（dB）signal to noise ratio in dB per band */
        SNR_Q7 = 3 * ( silk_lin2log( psSilk_VAD->NrgRatioSmth_Q8[b] ) - 8 * 128 );//=3*log2(psSilk_VAD->NrgRatioSmth_Q8[b]/2^8(去除Q8定标))*128(Q7定标)
        /* quality = sigmoid( 0.25 * ( SNR_dB - 16 ) ); */
        psEncC->input_quality_bands_Q15[ b ] = silk_sigm_Q15( silk_RSHIFT( SNR_Q7 - 16 * 128, 4 ) );
    }

    RESTORE_STACK;
    return( ret );
}

/**************************/
/* Noise level estimation */
/**************************/
# if  !defined(OPUS_X86_MAY_HAVE_SSE4_1)
static OPUS_INLINE
#endif
void silk_VAD_GetNoiseLevels(
    const opus_int32            pX[ VAD_N_BANDS ],  /* I    四个子带的能量subband energies                            */
    silk_VAD_state              *psSilk_VAD         /* I/O  Pointer to Silk VAD state                   */
)
{
    opus_int   k;
    opus_int32 nl, nrg, inv_nrg;
    opus_int   coef, min_coef;

    /* 前20秒滑动值较大，用于加速噪声收敛,刚开始平滑的很快Initially faster smoothing */
    if( psSilk_VAD->counter < 1000 ) { /* 1000 = 20 sec */
        //psSilk_VAD->counter初始化时设置成15，其初始值影响收敛时间，min_coef是噪声平滑的下限
        min_coef = silk_DIV32_16( silk_int16_MAX, silk_RSHIFT( psSilk_VAD->counter, 4 ) + 1 );//2^15 - 1 =  32767
        //32767 / ((psSilk_VAD->counter>> 4) + 1);
    } else {
        min_coef = 0;
    }
    //for 循环更新噪声水平, NL是noise level的缩写，inv_NL表示的是inverse noise level
    for( k = 0; k < VAD_N_BANDS; k++ ) //遍历四个子带
    {
        /* //获取历史噪声水平,获取当前频带的旧噪声级估计Get old noise level estimate for current band */
        nl = psSilk_VAD->NL[ k ];//5000，2500，1600，1200噪声门限
        silk_assert( nl >= 0 );

        /* 四个子带噪声初始偏移值设置成了自然环境中常见的粉红噪声，粉红噪声的功率谱和频率的倒数成比例，频率越低值越大[50，25，16，12],NL各子带的值为50*100， 25*100， 16*100， 12*100添加偏差Add bias */
        nrg = silk_ADD_POS_SAT32( pX[ k ], psSilk_VAD->NoiseLevelBias[ k ] );//pX[ k ]+[50，25，16，12],在自然噪声的基础上加上了每个子带的能量
        silk_assert( nrg > 0 );

        /* 能量的倒数Invert energies */
        inv_nrg = silk_DIV32( silk_int32_MAX, nrg );//=42949672,85899345,134217727, 178956970  0x7FFFFFFF / nrg   能量用32bit最大数除以能量，这么做的原因是能量按照整数计算的，为了防止溢出
        silk_assert( inv_nrg >= 0 );

        /* 子带能量高时更新较少Less update when subband energy is high */
        //根据当前能量和历史噪声能量大小情况，更新系数，更新噪声估计更新水平，能量高的子带更新的慢一些
        if( nrg > silk_LSHIFT( nl, 3 ) ) //nrg>40000
        {
            coef = VAD_NOISE_LEVEL_SMOOTH_COEF_Q16 >> 3;//128,噪声小
        } else if( nrg < nl ) //nrg<5000
        {
            coef = VAD_NOISE_LEVEL_SMOOTH_COEF_Q16;//=1024，噪声大
        } else //nrg再5000和40000之间，需要计算噪声水平系数
        {
            coef = silk_SMULWB( silk_SMULWW( inv_nrg, nl ), VAD_NOISE_LEVEL_SMOOTH_COEF_Q16 << 1 );//（(inv_nrg*nl)>>16   * 2048 ）>>16
        }

        /* 最初平滑速度更快Initially faster smoothing */
        //获取噪声水平,更新系数
        coef = silk_max_int( coef, min_coef );//=32767,32767,32767,32767

        /* 平滑能量的的倒数Smooth inverse energies */
        //平滑噪声 inv_nrg是当前噪声水平，inv_NL是历史噪声水平,inv_NL[ k ]=[429496,858993,1342177,1789569]
        psSilk_VAD->inv_NL[ k ] = silk_SMLAWB( psSilk_VAD->inv_NL[ k ], inv_nrg - psSilk_VAD->inv_NL[ k ], coef );//=psSilk_VAD->inv_NL[k] + (inv_nrg - psSilk_VAD->inv_NL[k] * coef) >> 16;
        silk_assert( psSilk_VAD->inv_NL[ k ] >= 0 );                                              //[21688935,43377871,67777924,90370566]

        /* // 在倒数域计算完成后，在转换回正常的线性域 ,计算噪声级别通过再次取倒数Compute noise level by inverting again */
        nl = silk_DIV32( silk_int32_MAX, psSilk_VAD->inv_NL[ k ] );//=99,49,31,23
        silk_assert( nl >= 0 );

        /* //限制噪声水平大小，确保高7个比特没被使用,限制噪音水平（保证7位）Limit noise levels (guarantee 7 bits of head room) */
        nl = silk_min( nl, 0x00FFFFFF );

        /* 存储更新的噪声水平,状态存储Store as part of state */
        psSilk_VAD->NL[ k ] = nl;//[99,49,31,23]
    }

    /* 递增帧计数器Increment frame counter */
    psSilk_VAD->counter++;
}
