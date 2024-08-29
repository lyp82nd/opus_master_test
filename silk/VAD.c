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
    silk_VAD_state              *psSilk_VAD                     /* I/O  Pointer to Silk VAD state               ����&psEnc->sCmn.sVAD    */
)
{//VAD��ʼ��������ʼ���������Ļ�ֵҲ����Ĭ�ϱ��������Ƿۺ��������������ȼ�ƫ�Ʊ������ڣ���ͬʱҲ��ʼ���������ȼ��ĵ�������ʼ����֡��������ֵ����Ϊ15���ڼ���ƽ��������ʱ����Ҫ�õ�֡��������¼VAD�����֡��������ƽ�������Ĳ�������֤ǰ1000֡��20s���������ʹ����ƽ���ĺܿ죬��ʼ����ÿ���Ӵ�������������ֵĬ��20dbҲ�����ź�������������100��
    opus_int b, ret = 0;

    /* reset state memory */
    silk_memset( psSilk_VAD, 0, sizeof( silk_VAD_state ) );//���ṹ������

    /* init noise levels */
    /* Initialize array with approx pink noise levels (psd proportional to inverse of frequency) */
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NoiseLevelBias[ b ] = silk_max_32( silk_DIV32_16( VAD_NOISE_LEVELS_BIAS, b + 1 ), 1 );//�ۺ�������ƫ����ֵ��50��25��16��12��
    }

    /* Initialize state */
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NL[ b ]     = silk_MUL( 100, psSilk_VAD->NoiseLevelBias[ b ] );//������������Ϊ�ۺ���������
        psSilk_VAD->inv_NL[ b ] = silk_DIV32( silk_int32_MAX, psSilk_VAD->NL[ b ] );//��ת����
    }
    psSilk_VAD->counter = 15;                                                       //֡������                  ����ͳ��VAD��֡����20msΪһ֡���ڿ�ʼ����Ϊ15��������Ҫ����֡������������ƽ��ϵ����ǰ20sҲ����ǰ1000֡��Ҫ�ܿ��ƽ������

    /* ��ʼ���Ӵ�ƽ������������ init smoothed energy-to-noise ratio*/
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        psSilk_VAD->NrgRatioSmth_Q8[ b ] = 100 * 256;       /* 100 * 256 --> 20 dB SNR */     //ÿ��Ƶ���е�����������25600->20dbSNR���������ڼ���ÿ��Ƶ���������
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
    opus_int32 Xnrg[ VAD_N_BANDS ];//��¼��ÿ���Ӵ�������
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
    /* �˲��ͷ���Filter and Decimate */
    /***********************/
    decimated_framelength1 = silk_RSHIFT( psEncC->frame_length, 1 );//80��frame_length=160������֮֡
    decimated_framelength2 = silk_RSHIFT( psEncC->frame_length, 2 );//40���ķ�֮֡
    decimated_framelength = silk_RSHIFT( psEncC->frame_length, 3 );//20���˷�֮֡
    /* Decimate into 4 bands:
       0       L      3L       L              3L                             5L
               -      --       -              --                             --
               8       8       2               4                              4

       [0-1 kHz| temp. |1-2 kHz|    2-4 kHz    |            4-8 kHz           |

       ���Ǳ�����Ϊ���²���������������С�ģ�frame_length/4�����⻮�ֵĿռ�They're arranged to allow the minimal ( frame_length / 4 ) extra
       scratch space during the downsampling process */
    X_offset[ 0 ] = 0;//decimated_framelength=20
    X_offset[ 1 ] = decimated_framelength + decimated_framelength2;//60
    X_offset[ 2 ] = X_offset[ 1 ] + decimated_framelength;//80
    X_offset[ 3 ] = X_offset[ 2 ] + decimated_framelength2;/*X_offset=0��60��80��120,�������X_offset�ֱ��¼��0-1kHz��1-2kHz��2-4kHz��4-8kHz�ĸ��Ӵ�ʱ������ֹƫ��λ��*/
    ALLOC( X, X_offset[ 3 ] + decimated_framelength1, opus_int16 );//�����������Ӵ�����ʱ���õ�
    //silk_ana_filt_bank_1������ʵ�ֳ�ȡ�²����������������Q8����
    /* 0-8 kHz to 0-4 kHz and 4-8 kHz */
    silk_ana_filt_bank_1( pIn, &psSilk_VAD->AnaState[  0 ],
        X, &X[ X_offset[ 3 ] ], psEncC->frame_length );

    /* 0-4 kHz to 0-2 kHz and 2-4 kHz */
    silk_ana_filt_bank_1( X, &psSilk_VAD->AnaState1[ 0 ],
        X, &X[ X_offset[ 2 ] ], decimated_framelength1 );

    /* 0-2 kHz to 0-1 kHz and 1-2 kHz */
    silk_ana_filt_bank_1( X, &psSilk_VAD->AnaState2[ 0 ],
        X, &X[ X_offset[ 1 ] ], decimated_framelength2 );

    //decimated_framelength�ĳ�����40�������Ƶ������Ӵ�ʹ�ø�ͨ�˲���
//������Ϊ�˵ķ���һ���ڼ�ʮ�������ϣ����ԶԵ�Ƶ�źŽ����ʵ������������ķ�ʽ�Ǽ�ȥǰһ������
    /*********************************************/
    /* ���Ƶ���ϵ�HP�˲�����΢������HP filter on lowest band (differentiator) */
    /*********************************************/
    X[ decimated_framelength - 1 ] = silk_RSHIFT( X[ decimated_framelength - 1 ], 1 );//����Ӵ������һ��������һλ
    HPstateTmp = X[ decimated_framelength - 1 ];//��ͨ�˲���״̬
    for( i = decimated_framelength - 1; i > 0; i-- ) {
        X[ i - 1 ]  = silk_RSHIFT( X[ i - 1 ], 1 );
        X[ i ]     -= X[ i - 1 ];
    }
    X[ 0 ] -= psSilk_VAD->HPstate;
    psSilk_VAD->HPstate = HPstateTmp;//��ͨ�˲���״̬

    /*************************************/
    /* ����ÿ���Ӵ�������Calculate the energy in each band */
    /*************************************/
    for( b = 0; b < VAD_N_BANDS; b++ ) //�����ĸ��Ӵ�
    {
        /* �����������Ӵ�λ�ã�������Ӵ�ʱ��������Ӵ�0����20���ڷǾ��Ȼ��ֵ�Ƶ���в��ҳ�ȡ��֡����Find the decimated framelength in the non-uniformly divided bands */
        decimated_framelength = silk_RSHIFT( psEncC->frame_length, silk_min_int( VAD_N_BANDS - b, VAD_N_BANDS - 1 ) );//��ǰ�Ӵ��ĳ��ȣ�silk_min_int( VAD_N_BANDS - b, VAD_N_BANDS - 1 )ֻ��ȷ���������

        /* �����Ӵ���ʱ����������ÿһ���Ӵ��ٷֳ��ķ���֡Split length into subframe lengths */
        dec_subframe_length = silk_RSHIFT( decimated_framelength, VAD_INTERNAL_SUBFRAMES_LOG2 );//b=0,=5
        dec_subframe_offset = 0;

        /* ����ÿ����֡������Compute energy per sub-frame */
        /* ���Ӵ���������ʼֵΪǰһ���Ӵ�������ֵ�������һ����֡�������ܺͳ�ʼ��initialize with summed energy of last subframe */
        Xnrg[ b ] = psSilk_VAD->XnrgSubfr[ b ];//0,
        for( s = 0; s < VAD_INTERNAL_SUBFRAMES; s++ ) //����ÿ���Ӵ����ĸ���֡
        {
            sumSquared = 0;                                                          //��¼���Ƿֳ��ķ����ݺ�ÿһ����������
            //ǰ��֡����������ÿ��������������������ӵ�һ�����һ֡��������ÿ���������������ͺ�����һλsumSquared >> 1���ټӵ�һ��
            for( i = 0; i < dec_subframe_length; i++ ) //����ÿ����֡�е�ÿ������
            {
                /* ������С��dec_subframe_length*��silk_int16_MIN/8��^2��The energy will be less than dec_subframe_length * ( silk_int16_MIN / 8 ) ^ 2.            */
                /* ��ˣ����ǿ�����û��������յ�����½����ۻ�������dec_subframe_length>128��Therefore we can accumulate with no risk of overflow (unless dec_subframe_length > 128)  */
                x_tmp = silk_RSHIFT(X[ X_offset[ b ] + i + dec_subframe_offset ], 3 );//������Ƶ����������λ�൱��/8
                sumSquared = silk_SMLABB( sumSquared, x_tmp, x_tmp );//(sumSquared+x_tmp * x_tmp/64)

                /* Safety check */
                silk_assert( sumSquared >= 0 );
            }

            /*Add/saturate summed energy of current subframe */
            if( s < VAD_INTERNAL_SUBFRAMES - 1 ) {
                Xnrg[b] = silk_ADD_POS_SAT32(Xnrg[b], sumSquared);//(Xnrg[b] + sumSquared)                                       //���Ӵ�������¼��Xnrg��
            } else {
                /* Look-ahead subframe */
                Xnrg[ b ] = silk_ADD_POS_SAT32( Xnrg[ b ], silk_RSHIFT( sumSquared, 1 ) );//Xnrg[b] + sumSquared >> 1            //���һ������/2�ټ��뵽��ǰ�Ӵ�������ȥ
            }

            dec_subframe_offset += dec_subframe_length;//�����±��ƶ�
        }
        psSilk_VAD->XnrgSubfr[ b ] = sumSquared;                                                            //���Ӵ����һ�����ݵ�������Ϊ��һ֡��������һ�Ӵ����ĳ�ʼֵ
    }

    /********************/
    /* �������Noise estimation */
    /********************/
    silk_VAD_GetNoiseLevels( &Xnrg[ 0 ], psSilk_VAD );

    /***********************************************/
    /* �����SNR Signal-plus-noise to noise ratio estimation */
    /***********************************************/
    sumSquared = 0;
    input_tilt = 0;
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        speech_nrg = Xnrg[ b ] - psSilk_VAD->NL[ b ];//�������źŵ�����=�ź�����-�����ȼ�����������Ƿ���������
        if( speech_nrg > 0 ) {
            /* Divide, with sufficient resolution */
            if( ( Xnrg[ b ] & 0xFF800000 ) == 0 ) //�߾�λû��ֵ
            {
                NrgToNoiseRatio_Q8[ b ] = silk_DIV32( silk_LSHIFT( Xnrg[ b ], 8 ), psSilk_VAD->NL[ b ] + 1 );//���¿�����ֵ�Ķ���λ�ã����������ȼ���һȡ����֮���������
            } 
            else {
                NrgToNoiseRatio_Q8[ b ] = silk_DIV32( Xnrg[ b ], silk_RSHIFT( psSilk_VAD->NL[ b ], 8 ) + 1 );
            }
            //���¿��Ǿ��ȣ�������snr
            /* ת����������Convert to log domain */
            SNR_Q7 = silk_lin2log(NrgToNoiseRatio_Q8[b]) - 8 * 128;//128log(Xb)-8*128=2^7 *log(Xb/2^8)���ڲ���Q8��ʽ����������128�õ�Q7��ʽ����

            /* ƽ����Sum-of-squares */
            sumSquared = silk_SMLABB( sumSquared, SNR_Q7, SNR_Q7 );          /* Q14 */

            /* ��б����Tilt measure */
            if( speech_nrg < ( (opus_int32)1 << 20 ) ) {
                /* С���Ӵ���������������SNRֵScale down SNR value for small subband speech energies */
                SNR_Q7 = silk_SMULWB( silk_LSHIFT( silk_SQRT_APPROX( speech_nrg ), 6 ), SNR_Q7 );
            }
            input_tilt = silk_SMLAWB(input_tilt, tiltWeights[b], SNR_Q7);//����б���㣬input_tilt+�����Ӵ���(tiltWeights*SNR)
        } else {
            NrgToNoiseRatio_Q8[ b ] = 256;
        }
    }

    /* ƽ����ƽ��ֵMean-of-squares */
    sumSquared = silk_DIV32_16( sumSquared, VAD_N_BANDS ); /* Q14 */

    /* ���������ƣ����ŵ�dBs����д�����ָ��Root-mean-square approximation, scale to dBs, and write to output pointer */
    pSNR_dB_Q7 = (opus_int16)( 3 * silk_SQRT_APPROX( sumSquared ) ); /* Q7 */

    /*********************************/
    /* �������ʹ���Speech Probability Estimation */
    /*********************************/
    SA_Q15 = silk_sigm_Q15(silk_SMULWB(VAD_SNR_FACTOR_Q16, pSNR_dB_Q7) - VAD_NEGATIVE_OFFSET_Q5);//((45000*pSNR_dB_Q7)>>16  -128=-128)���=589

    /**************************/
    /*Ƶ����б���� Frequency Tilt Measure */
    /**************************/
    psEncC->input_tilt_Q15 = silk_LSHIFT( silk_sigm_Q15( input_tilt ) - 16384, 1 );

    /**************************************************/
    /* ���������������������ʽ�������Scale the sigmoid output based on power levels */
    /**************************************************/
    speech_nrg = 0;
    for( b = 0; b < VAD_N_BANDS; b++ ) {
        /* ����û�������������źţ����ߵ�Ƶ�����и����Ȩ��Accumulate signal-without-noise energies, higher frequency bands have more weight */
        speech_nrg += ( b + 1 ) * silk_RSHIFT( Xnrg[ b ] - psSilk_VAD->NL[ b ], 4 );//���ڸ�Ƶ�Ӵ�����С������������ˣ�b+1����Ȩ����Ϊ��4���Ӵ�������ÿ���Ӵ���������������4�Ի���ĸ��ִ���ƽ��ֵ
    }

    if( psEncC->frame_length == 20 * psEncC->fs_kHz ) {//��20ms֡�����������2�Եõ�10ms֡���������������������һλʵ�ֳ�2������
        speech_nrg = silk_RSHIFT32( speech_nrg, 1 );
    }
    /* //����ǰ������������������������Power scaling */
    if( speech_nrg <= 0 ) {//���С�ڵ����㣬˵��ǰ���ڼ�Ȩ����������ʱ������ˣ���ʱ��СSA_Q15��ֵ
        SA_Q15 = silk_RSHIFT( SA_Q15, 1 );
    } else if( speech_nrg < 16384 ) {
        speech_nrg = silk_LSHIFT32( speech_nrg, 16 );
        //����������֮��ת����Q15��ʾ��32768=2^15 ��
        /* square-root */
        speech_nrg = silk_SQRT_APPROX( speech_nrg );
        SA_Q15 = silk_SMULWB( 32768 + speech_nrg, SA_Q15 );
        //((((32768 + speech_nrg)* ( SA_Q15)) >> 16));////////////////////////////////////////////////
    }

    /* ��¼���ݣ���ʽת��ΪQ8Copy the resulting speech activity in Q8 */
    psEncC->speech_activity_Q8 = silk_min_int( silk_RSHIFT( SA_Q15, 7 ), silk_uint8_MAX );

    /***********************************/
    /* ������������ȹ���Energy Level and SNR estimation */
    /***********************************/
    /* ƽ�����������������ʼ������Smoothing coefficient */
    smooth_coef_Q16 = silk_SMULWB( VAD_SNR_SMOOTH_COEF_Q18, silk_SMULWB( (opus_int32)SA_Q15, SA_Q15 ) );//(4096*(SA_Q15* SA_Q15) >> 16) >> 16;

    if( psEncC->frame_length == 10 * psEncC->fs_kHz ) {
        smooth_coef_Q16 >>= 1;
    }//�����10ms��ƽ��ϵ������һ��

    for( b = 0; b < VAD_N_BANDS; b++ ) {
        /* ����ÿ���Ӵ�ƽ������������compute smoothed energy-to-noise ratio per band */
        psSilk_VAD->NrgRatioSmth_Q8[ b ] = silk_SMLAWB( psSilk_VAD->NrgRatioSmth_Q8[ b ],NrgToNoiseRatio_Q8[ b ] - psSilk_VAD->NrgRatioSmth_Q8[ b ], smooth_coef_Q16 );
                                                     //psSilk_VAD->NrgRatioSmth_Q8[b] + ((NrgToNoiseRatio_Q8[b] - psSilk_VAD->NrgRatioSmth_Q8[b]) *smooth_coef_Q16) >> 16;
        /* ÿ��Ƶ��������ȣ�dB��signal to noise ratio in dB per band */
        SNR_Q7 = 3 * ( silk_lin2log( psSilk_VAD->NrgRatioSmth_Q8[b] ) - 8 * 128 );//=3*log2(psSilk_VAD->NrgRatioSmth_Q8[b]/2^8(ȥ��Q8����))*128(Q7����)
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
    const opus_int32            pX[ VAD_N_BANDS ],  /* I    �ĸ��Ӵ�������subband energies                            */
    silk_VAD_state              *psSilk_VAD         /* I/O  Pointer to Silk VAD state                   */
)
{
    opus_int   k;
    opus_int32 nl, nrg, inv_nrg;
    opus_int   coef, min_coef;

    /* ǰ20�뻬��ֵ�ϴ����ڼ�����������,�տ�ʼƽ���ĺܿ�Initially faster smoothing */
    if( psSilk_VAD->counter < 1000 ) { /* 1000 = 20 sec */
        //psSilk_VAD->counter��ʼ��ʱ���ó�15�����ʼֵӰ������ʱ�䣬min_coef������ƽ��������
        min_coef = silk_DIV32_16( silk_int16_MAX, silk_RSHIFT( psSilk_VAD->counter, 4 ) + 1 );//2^15 - 1 =  32767
        //32767 / ((psSilk_VAD->counter>> 4) + 1);
    } else {
        min_coef = 0;
    }
    //for ѭ����������ˮƽ, NL��noise level����д��inv_NL��ʾ����inverse noise level
    for( k = 0; k < VAD_N_BANDS; k++ ) //�����ĸ��Ӵ�
    {
        /* //��ȡ��ʷ����ˮƽ,��ȡ��ǰƵ���ľ�����������Get old noise level estimate for current band */
        nl = psSilk_VAD->NL[ k ];//5000��2500��1600��1200��������
        silk_assert( nl >= 0 );

        /* �ĸ��Ӵ�������ʼƫ��ֵ���ó�����Ȼ�����г����ķۺ��������ۺ������Ĺ����׺�Ƶ�ʵĵ����ɱ�����Ƶ��Խ��ֵԽ��[50��25��16��12],NL���Ӵ���ֵΪ50*100�� 25*100�� 16*100�� 12*100���ƫ��Add bias */
        nrg = silk_ADD_POS_SAT32( pX[ k ], psSilk_VAD->NoiseLevelBias[ k ] );//pX[ k ]+[50��25��16��12],����Ȼ�����Ļ����ϼ�����ÿ���Ӵ�������
        silk_assert( nrg > 0 );

        /* �����ĵ���Invert energies */
        inv_nrg = silk_DIV32( silk_int32_MAX, nrg );//=42949672,85899345,134217727, 178956970  0x7FFFFFFF / nrg   ������32bit�����������������ô����ԭ��������������������ģ�Ϊ�˷�ֹ���
        silk_assert( inv_nrg >= 0 );

        /* �Ӵ�������ʱ���½���Less update when subband energy is high */
        //���ݵ�ǰ��������ʷ����������С���������ϵ���������������Ƹ���ˮƽ�������ߵ��Ӵ����µ���һЩ
        if( nrg > silk_LSHIFT( nl, 3 ) ) //nrg>40000
        {
            coef = VAD_NOISE_LEVEL_SMOOTH_COEF_Q16 >> 3;//128,����С
        } else if( nrg < nl ) //nrg<5000
        {
            coef = VAD_NOISE_LEVEL_SMOOTH_COEF_Q16;//=1024��������
        } else //nrg��5000��40000֮�䣬��Ҫ��������ˮƽϵ��
        {
            coef = silk_SMULWB( silk_SMULWW( inv_nrg, nl ), VAD_NOISE_LEVEL_SMOOTH_COEF_Q16 << 1 );//��(inv_nrg*nl)>>16   * 2048 ��>>16
        }

        /* ���ƽ���ٶȸ���Initially faster smoothing */
        //��ȡ����ˮƽ,����ϵ��
        coef = silk_max_int( coef, min_coef );//=32767,32767,32767,32767

        /* ƽ�������ĵĵ���Smooth inverse energies */
        //ƽ������ inv_nrg�ǵ�ǰ����ˮƽ��inv_NL����ʷ����ˮƽ,inv_NL[ k ]=[429496,858993,1342177,1789569]
        psSilk_VAD->inv_NL[ k ] = silk_SMLAWB( psSilk_VAD->inv_NL[ k ], inv_nrg - psSilk_VAD->inv_NL[ k ], coef );//=psSilk_VAD->inv_NL[k] + (inv_nrg - psSilk_VAD->inv_NL[k] * coef) >> 16;
        silk_assert( psSilk_VAD->inv_NL[ k ] >= 0 );                                              //[21688935,43377871,67777924,90370566]

        /* // �ڵ����������ɺ���ת���������������� ,������������ͨ���ٴ�ȡ����Compute noise level by inverting again */
        nl = silk_DIV32( silk_int32_MAX, psSilk_VAD->inv_NL[ k ] );//=99,49,31,23
        silk_assert( nl >= 0 );

        /* //��������ˮƽ��С��ȷ����7������û��ʹ��,��������ˮƽ����֤7λ��Limit noise levels (guarantee 7 bits of head room) */
        nl = silk_min( nl, 0x00FFFFFF );

        /* �洢���µ�����ˮƽ,״̬�洢Store as part of state */
        psSilk_VAD->NL[ k ] = nl;//[99,49,31,23]
    }

    /* ����֡������Increment frame counter */
    psSilk_VAD->counter++;
}
