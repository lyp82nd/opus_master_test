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
    /* I:（API通道数）指应用程序向编解码器提供的音频数据的通道数。Number of channels; 1/2                                                         */
    opus_int32 nChannelsAPI;

    /* I:   （物理通道数）编解码器实际处理的音频流的通道数Number of channels; 1/2                                                         */
    opus_int32 nChannelsInternal;

    /* I:传入opus的音频采样率。Input signal sampling rate in Hertz; 8000/12000/16000/24000/32000/44100/48000   */
    opus_int32 API_sampleRate;

    /* I:   最大Opus内部采样率。会最后在silk_control_encoder函数中进行判断，当前采样率是否大于此值Maximum internal sampling rate in Hertz; 8000/12000/16000                       */
    opus_int32 maxInternalSampleRate;

    /* I:   最小Opus内部采样率，最后在silk_control_encoder函数中进行判断，当前采样率是否大于此值Minimum internal sampling rate in Hertz; 8000/12000/16000                       */
    opus_int32 minInternalSampleRate;

    /* I:   期望Opus编解码器内部使用的采样率。后面会根据带宽选择采样率，当前带宽是窄带则采用8k，在silk_control_encoder函数用来判断是否需要上采样或者下采样的参数Soft request for internal sampling rate in Hertz; 8000/12000/16000              */
    opus_int32 desiredInternalSampleRate;

    /* I:   每个Opus数据包的时长（以毫秒为单位)在silk编码过程中会计算的和framesize相匹配的ms数，与本身初值无关，Number of samples per packet in milliseconds; 10/20/40/60                       */
    opus_int payloadSize_ms;

    /* I:   压缩码流目标速率bps，后面根据这个值计算每一帧压缩后输出的位数 Bitrate during active speech in bits/second; internally limited                 */
    opus_int32 bitRate;

    /* I:   上行数据包丢失百分比(0-100)Uplink packet loss in percent (0-100)                                           */
    opus_int packetLossPercentage;

    /*1-10，指在opus内部运算的复杂度，会决定实际比特率取(90+complexity)/100倍的比特率，数值分区间的决定基频估计器的复杂度，基频估计器的阙值，基频分析中白化滤波器的阶数，噪声整形滤波器的阶数//基频分析学习样本个数，量化判决的状态数量，是否采用NLSF，NLSF_MSVQ剩余的个数，扭曲噪声整形的扭曲参数初值 I:Complexity mode; 0 is lowest, 10 is highest complexity                          */
    opus_int complexity;

    /* I:  强制启用带内前向纠错(FEC)的标志 Flag to enable in-band Forward Error Correction (FEC); 0/1 它是一种在音频通信中用于提高抗丢包能力的技术。当音频数据在传输过程中发生了丢包，FEC可以通过在发送端添加冗余信息，在接收端利用这些冗余信息进行恢复，从而减少音频数据的丢失。
具体来说，Opus中的FEC采用的是基于包的FEC技术，即在每个音频包中添加一定量的冗余信息，以便在丢包时进行恢复。Opus中支持两种FEC模式：
补偿模式1：在发送端对每个音频包添加一定数量的冗余数据，接收端在收到丢失的数据包时使用这些冗余数据进行恢复。补偿模式适用于低延迟应用场景，但需要额外的带宽来传输冗余数据。
独立模式0：在发送端将音频数据分为多个独立的流，并在每个流中添加一定数量的冗余数据。接收端在收到丢失的数据包时，可以从其他流中获取冗余数据进行恢复。独立模式适用于高延迟应用场景，可以提供更好的恢复效果。=0                   */
    opus_int useInBandFEC;

    /* I:   Flag to actually code in-band Forward Error Correction (FEC) in the current packet; 0/1 */
    opus_int LBRR_coded;

    /* I:   0则不检测静音段，1则检测静音段，超过400ms则单独发一帧标志此静音段（Discontinuous Transmission，不连续传输）Flag to enable discontinuous transmission (DTX); 0/1                            */
    opus_int useDTX;

    /* I:   设置基于SILK算法的编码器是否启用CBR（Constant Bitrate，定码率）模式。启用CBR模式可能会导致音频质量下降（特别是在较低的比特率下），因为编码器必须保持固定的比特率，无法根据语音内容的复杂度进行自适应调整。Flag to use constant bitrate                                                    */
    opus_int useCBR;

    /* I:  允许帧的最大位数 Maximum number of bits allowed for the frame                                    */
    opus_int maxBits;

    /* I:   平滑的混音到单声道Causes a smooth downmix to mono                                                 */
    opus_int toMono;

    /* I:   Opus 编码器允许切换带宽Opus encoder is allowing us to switch bandwidth                                 */
    opus_int opusCanSwitch;

    /* I: 设置为0则帧与帧之间相关，设置为1则相互独立，会增加每一帧携带的信息增加码率Make frames as independent as possible (but still use LPC)                        */
    opus_int reducedDependency;

    /* O:   内部采样率Internal sampling rate used, in Hertz; 8000/12000/16000                         */
    opus_int32 internalSampleRate;

    /* O: 标记允许带宽切换(因为低语音活动)Flag that bandwidth switching is allowed (because low voice activity)             */
    opus_int allowBandwidthSwitch;

    /* O:   这个标志意味着SILK以宽带模式运行，没有使用可变的线性预测滤波器。它用于在宽带（WB）、超宽带（SWB）和全带（FB）之间进行切换。Flag that SILK runs in WB mode without variable LP filter (use for switching between WB/SWB/FB) */
    opus_int inWBmodeWithoutVariableLP;

    /* O:   立体声宽度Stereo width */
    opus_int stereoWidth_Q14;

    /* O:  需要切换使用Opus编码器来进行编码 Tells the Opus encoder we're ready to switch                                    */
    opus_int switchReady;

    /* O: 信号类型SILK Signal type */
    opus_int signalType;

    /* O: 减少SILK编码器中量化误差的抖动技术，并且可以通过调整抖动参数来控制SILK offset (dithering) */
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
