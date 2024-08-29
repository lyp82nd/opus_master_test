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

#ifndef SILK_CONTROL_H
#define SILK_CONTROL_H

#include "typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Decoder API flags */
#define FLAG_DECODE_NORMAL                      0
#define FLAG_PACKET_LOST                        1
#define FLAG_DECODE_LBRR                        2

/***********************************************/
/* Structure for controlling encoder operation */
/***********************************************/
typedef struct {
    /* I:��APIͨ������ָӦ�ó������������ṩ����Ƶ���ݵ�ͨ������Number of channels; 1/2                                                         */
    opus_int32 nChannelsAPI;

    /* I:   ������ͨ�������������ʵ�ʴ������Ƶ����ͨ����Number of channels; 1/2                                                         */
    opus_int32 nChannelsInternal;

    /* I:����opus����Ƶ�����ʡ�Input signal sampling rate in Hertz; 8000/12000/16000/24000/32000/44100/48000   */
    opus_int32 API_sampleRate;

    /* I:   ���Opus�ڲ������ʡ��������silk_control_encoder�����н����жϣ���ǰ�������Ƿ���ڴ�ֵMaximum internal sampling rate in Hertz; 8000/12000/16000                       */
    opus_int32 maxInternalSampleRate;

    /* I:   ��СOpus�ڲ������ʣ������silk_control_encoder�����н����жϣ���ǰ�������Ƿ���ڴ�ֵMinimum internal sampling rate in Hertz; 8000/12000/16000                       */
    opus_int32 minInternalSampleRate;

    /* I:   ����Opus��������ڲ�ʹ�õĲ����ʡ��������ݴ���ѡ������ʣ���ǰ������խ�������8k����silk_control_encoder���������ж��Ƿ���Ҫ�ϲ��������²����Ĳ���Soft request for internal sampling rate in Hertz; 8000/12000/16000              */
    opus_int32 desiredInternalSampleRate;

    /* I:   ÿ��Opus���ݰ���ʱ�����Ժ���Ϊ��λ)��silk��������л����ĺ�framesize��ƥ���ms�����뱾���ֵ�޹أ�Number of samples per packet in milliseconds; 10/20/40/60                       */
    opus_int payloadSize_ms;

    /* I:   ѹ������Ŀ������bps������������ֵ����ÿһ֡ѹ���������λ�� Bitrate during active speech in bits/second; internally limited                 */
    opus_int32 bitRate;

    /* I:   �������ݰ���ʧ�ٷֱ�(0-100)Uplink packet loss in percent (0-100)                                           */
    opus_int packetLossPercentage;

    /*1-10��ָ��opus�ڲ�����ĸ��Ӷȣ������ʵ�ʱ�����ȡ(90+complexity)/100���ı����ʣ���ֵ������ľ�����Ƶ�������ĸ��Ӷȣ���Ƶ����������ֵ����Ƶ�����а׻��˲����Ľ��������������˲����Ľ���//��Ƶ����ѧϰ���������������о���״̬�������Ƿ����NLSF��NLSF_MSVQʣ��ĸ�����Ť���������ε�Ť��������ֵ I:Complexity mode; 0 is lowest, 10 is highest complexity                          */
    opus_int complexity;

    /* I:  ǿ�����ô���ǰ�����(FEC)�ı�־ Flag to enable in-band Forward Error Correction (FEC); 0/1 ����һ������Ƶͨ����������߿����������ļ���������Ƶ�����ڴ�������з����˶�����FEC����ͨ���ڷ��Ͷ����������Ϣ���ڽ��ն�������Щ������Ϣ���лָ����Ӷ�������Ƶ���ݵĶ�ʧ��
������˵��Opus�е�FEC���õ��ǻ��ڰ���FEC����������ÿ����Ƶ�������һ������������Ϣ���Ա��ڶ���ʱ���лָ���Opus��֧������FECģʽ��
����ģʽ1���ڷ��Ͷ˶�ÿ����Ƶ�����һ���������������ݣ����ն����յ���ʧ�����ݰ�ʱʹ����Щ�������ݽ��лָ�������ģʽ�����ڵ��ӳ�Ӧ�ó���������Ҫ����Ĵ����������������ݡ�
����ģʽ0���ڷ��Ͷ˽���Ƶ���ݷ�Ϊ�����������������ÿ���������һ���������������ݡ����ն����յ���ʧ�����ݰ�ʱ�����Դ��������л�ȡ�������ݽ��лָ�������ģʽ�����ڸ��ӳ�Ӧ�ó����������ṩ���õĻָ�Ч����=0                   */
    opus_int useInBandFEC;

    /* I:   Flag to actually code in-band Forward Error Correction (FEC) in the current packet; 0/1 */
    opus_int LBRR_coded;

    /* I:   0�򲻼�⾲���Σ�1���⾲���Σ�����400ms�򵥶���һ֡��־�˾����Σ�Discontinuous Transmission�����������䣩Flag to enable discontinuous transmission (DTX); 0/1                            */
    opus_int useDTX;

    /* I:   ���û���SILK�㷨�ı������Ƿ�����CBR��Constant Bitrate�������ʣ�ģʽ������CBRģʽ���ܻᵼ����Ƶ�����½����ر����ڽϵ͵ı������£�����Ϊ���������뱣�̶ֹ��ı����ʣ��޷������������ݵĸ��ӶȽ�������Ӧ������Flag to use constant bitrate                                                    */
    opus_int useCBR;

    /* I:  ����֡�����λ�� Maximum number of bits allowed for the frame                                    */
    opus_int maxBits;

    /* I:   ƽ���Ļ�����������Causes a smooth downmix to mono                                                 */
    opus_int toMono;

    /* I:   Opus �����������л�����Opus encoder is allowing us to switch bandwidth                                 */
    opus_int opusCanSwitch;

    /* I: ����Ϊ0��֡��֮֡����أ�����Ϊ1���໥������������ÿһ֡Я������Ϣ��������Make frames as independent as possible (but still use LPC)                        */
    opus_int reducedDependency;

    /* O:   �ڲ�������Internal sampling rate used, in Hertz; 8000/12000/16000                         */
    opus_int32 internalSampleRate;

    /* O: �����������л�(��Ϊ�������)Flag that bandwidth switching is allowed (because low voice activity)             */
    opus_int allowBandwidthSwitch;

    /* O:   �����־��ζ��SILK�Կ��ģʽ���У�û��ʹ�ÿɱ������Ԥ���˲������������ڿ����WB�����������SWB����ȫ����FB��֮������л���Flag that SILK runs in WB mode without variable LP filter (use for switching between WB/SWB/FB) */
    opus_int inWBmodeWithoutVariableLP;

    /* O:   ���������Stereo width */
    opus_int stereoWidth_Q14;

    /* O:  ��Ҫ�л�ʹ��Opus�����������б��� Tells the Opus encoder we're ready to switch                                    */
    opus_int switchReady;

    /* O: �ź�����SILK Signal type */
    opus_int signalType;

    /* O: ����SILK���������������Ķ������������ҿ���ͨ��������������������SILK offset (dithering) */
    opus_int offset;
} silk_EncControlStruct;

/**************************************************************************/
/* Structure for controlling decoder operation and reading decoder status */
/**************************************************************************/
typedef struct {
    /* I:   Number of channels; 1/2                                                         */
    opus_int32 nChannelsAPI;

    /* I:   Number of channels; 1/2                                                         */
    opus_int32 nChannelsInternal;

    /* I:   Output signal sampling rate in Hertz; 8000/12000/16000/24000/32000/44100/48000  */
    opus_int32 API_sampleRate;

    /* I:   Internal sampling rate used, in Hertz; 8000/12000/16000                         */
    opus_int32 internalSampleRate;

    /* I:   Number of samples per packet in milliseconds; 10/20/40/60                       */
    opus_int payloadSize_ms;

    /* O:   Pitch lag of previous frame (0 if unvoiced), measured in samples at 48 kHz      */
    opus_int prevPitchLag;
} silk_DecControlStruct;

#ifdef __cplusplus
}
#endif

#endif
