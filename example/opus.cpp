#include <iostream>
#include <opus_types.h>
#include  <opus.h>
#include <cstring>
#include <memory>

#include <vector>
// https://github.com/mackron/dr_libs/blob/master/dr_wav.h
// 
//计时所需头文件
#include <time.h>
#include <chrono>

using namespace std;
using namespace std::chrono;
//
#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"//wav库函数

//通常取值为2.5ms、5ms、10ms、20ms、40ms或60ms，对应的采样数分别为120、240、480、960、1920和2880。
//2.5ms（120采样）：适用于极低延迟的实时通信场景，例如在线游戏、视频会议等。(不行会报错)
//5ms（240采样）：适用于低延迟的实时通信场景，例如VoIP电话、网络广播等。(不行会报错)
//10ms（480采样）：适用于普通的实时通信场景，例如视频会议、语音聊天等。
//20ms（960采样）：适用于需要平衡延迟和音频质量的场景，例如在线教育、远程协作等。
//40ms（1920采样）：适用于强调音频质量的场景，例如语音识别、语音存储等。(不行会报错)
//60ms（2880采样）：适用于高品质音频的场景，例如音乐流媒体、录音等。(不行会报错)
#define FRAME_SIZE 80     //帧大小算法=fs*（多少ms为一帧），如果8k采样率，10ms为一帧则帧大小等于80，官方文件中说20ms为一帧，其中分为四个子帧5ms，所以输入采样率为8k，20ms为一帧则帧大小为8k*20ms=160,但是输出比特率太大了
#define MAX_FRAME_SIZE (6*FRAME_SIZE)//

#define MAX_CHANNELS 1
//一帧数据最多压缩成3*1276个字节
#define MAX_PACKET_SIZE (3*1276)

#pragma pack(push)
#pragma pack(1)

//本结构体定义为一个文件信息，包含三个属性分别是声道数和采样率和采样位数
struct WavInfo {
    uint16_t channels;//声道数
    uint32_t sampleRate;//采样率
    uint32_t bitsPerSample;//采样位数
};

#pragma pack(pop)
//如果指针没有被定义过，定义为空指针
#ifndef  nullptr
#define  nullptr NULL
#endif

class FileStream {
public:
    FileStream() {//无参构造函数，初始化 cur_pos = 0是记录当前位置的参数为0
        cur_pos = 0;
    }
    //Append定义，作用：从date里面复制size个字节到vec.data() + cur_pos中，同时cur_pos更新至size+cur_pos位置
    void Append(const char *data, size_t size) {
        if (cur_pos + size > Size()) {
            vec.resize(cur_pos + size);
        }
        memcpy(vec.data() + cur_pos, data, size);//从date里面复制size个字节到vec.data() + cur_pos中
        cur_pos += size;
    }

    void AppendU32(uint32_t val) {
        Append((char *) (&val), sizeof(val));
    }

    char *Data() {
        return vec.data();
    }
//返回向量组的数据数量
    size_t Size() {
        return vec.size();
    }

    //将文件信息从vec容器的前elemCount*elemSize个位置中读到buff中，并且返回read=1，cur_pos=cur_pos+elemCount
    size_t Read(void *buff, size_t elemSize, size_t elemCount) {
        size_t readed = std::min((vec.size() - cur_pos), (elemCount * elemSize)) / elemSize;
        if (readed > 0) {
            memcpy(buff, vec.data() + cur_pos, readed * elemSize);
                    //把vec.data() + cur_pos中的数据复制readed * elemSize个给buff
            cur_pos += readed * elemSize; 
                    //cur_pos更新cur_pos=cur_pos+readed * elemSize（cur_pos=cur_pos+10）
        }
        return readed;
    }

    //向量组vec是空的cur_pos=0，如不空cur_pos=（vec中数据量-1）的值，但基本cur_pos=0
    bool SeekCur(int offset) {
        //offset=0且cur_pos=0
        if (cur_pos + offset > vec.size()) {
            cur_pos = !vec.empty() ? (vec.size() - 1) : 0;//vec.empty()是来判断向量是否为空的
            return false;                                 //vec.size()是向量vec的数据量
        } else {
            cur_pos += offset;
            return true;
        }
    }
    //如果没有参数输入则cur_pos = 0
    bool SeekBeg(int offset = 0) {
        cur_pos = 0;
        return SeekCur(offset);
    }

    bool WriteToFile(const char *filename) {
        FILE *fin = fopen(filename, "wb");//创建输出文件，指向该流的文件指针被返回，创建不成功返回NULL 
        if (!fin) {
            return false;
        }
        fseek(fin, 0, SEEK_SET);//指向文件初始位置
        fwrite(vec.data(), sizeof(char), vec.size(), fin);//存储在vec向量中的所有数据写入到文件中，这意味着vec中存放着最终的压缩数据流
        fclose(fin);
        return true;
    }

    bool ReadFromFile(const char *filename) {
        FILE *fin = fopen(filename, "rb");
        if (!fin) {
            return false;
        }
        fseek(fin, 0, SEEK_END);
        long fileSize = ftell(fin);
        vec.resize(static_cast<unsigned long long int>(fileSize));
        fseek(fin, 0, SEEK_SET);
        fread(vec.data(), sizeof(char), vec.size(), fin);
        fclose(fin);
        return true;
    }

private:
    std::vector<char> vec;//向量的定义
    size_t cur_pos;
};

bool Wav2Opus(FileStream *input, FileStream *output);

bool Opus2Wav(FileStream *input, FileStream *output);

bool wav2stream(char *input, FileStream *output);

bool stream2wav(FileStream *input, char *output);

//输出的wav格式默认为riff格式（normal wav）
bool wavWrite_int16(char *filename, int16_t *buffer, int sampleRate, uint32_t totalSampleCount) {
    drwav_data_format format = {};
    format.container = drwav_container_riff;     // <-- drwav_container_riff = normal WAV files, drwav_container_w64 = Sony Wave64.
    format.format = DR_WAVE_FORMAT_PCM;          // <-- Any of the DR_WAVE_FORMAT_* codes.
    format.channels = 1;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 16;
    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "ERROR\n");
            return false;
        }
        return true;
    }
    return false;
}
//返回一个wav数据流的首地址，*wavBuffer记录的是数据的首地址,
int16_t *wavRead_int16(char *filename, uint32_t *sampleRate, uint64_t *totalSampleCount) {
    unsigned int channels;//通道数
    int16_t *buffer = drwav_open_and_read_file_s16(filename, &channels, sampleRate, totalSampleCount);
    if (buffer == nullptr) {
        fprintf(stderr, "wav音频读取失败ERROR\n");
        return nullptr;
    }
    if (channels != 1) {
        drwav_free(buffer);
        buffer = nullptr;
        *sampleRate = 0;
        *totalSampleCount = 0;
        cout << "不是单声道音频opus.cpp179 \n" << endl;
    }
    return buffer;//返回的是地址
}

bool wav2stream(char *input, FileStream *output) {
    uint32_t sampleRate = 0;//采样率定义为0
    uint64_t totalSampleCount = 0;//总体的样本数量，字节为单位
    int16_t *wavBuffer = wavRead_int16(input, &sampleRate, &totalSampleCount);//返回一个int16读取的wav数据流，*wavBuffer记录的是地址
    if (wavBuffer == nullptr) return false;
    WavInfo info = {};

    //强制的定义了输出音频的格式
    info.bitsPerSample = 16;
    info.sampleRate = sampleRate;//采样率与输入一致
    info.channels = 1;
    cout << "输出音频文件格式(采样位数固定16位，采样率与输入一致，强制输出音频为单声道音频)" << "\n" << endl;
    output->SeekBeg();
    output->Append((char *) &info, sizeof(info));//将info信息内容写入到vec.date中
    output->Append((char *) wavBuffer, totalSampleCount * sizeof(int16_t));//把wav数据从wavbuff指针所指向位置写入向量vec.date中，由于wavbuffer强制转成char所以totalSampleCount需要把每个占两字节的数据分成两倍的数量才能正确写入
    //totalSampleCount * sizeof(int16_t)=112640,cur_pos=10(头文件信息）+112640（wav数据，两字节的数据56320，一共112640个字节的数据）
    free(wavBuffer);
    return true;
}

bool stream2wav(FileStream *input, char *output) {//input输入是解压缩的比特流，output输出是输出最后的音频
    WavInfo info = {};
    input->SeekBeg();
    size_t read = input->Read(&info, sizeof(info), 1);//读取解压缩后的头文件信息
    if (read != 1) {
        return false;
    }
    size_t totalSampleCount = (input->Size() - sizeof(info)) / 2;
    return wavWrite_int16(output, (int16_t *) (input->Data() + sizeof(info)), info.sampleRate,
                          static_cast<uint32_t>(totalSampleCount));                //输出的wav格式默认为RIFF格式（normal wav）采用pcm格式，单声道，16采样精度，采样率和读取出来的采样率一致
}

bool Wav2Opus(FileStream *input, FileStream *output) {
                         
    WavInfo in_info = {};
    input->SeekBeg();
    size_t read = input->Read(&in_info, sizeof(in_info), 1);                   //从vec容器中读取头文件信息写给结构体in_info
    if (read != 1) {
        cout << "read!=1     文件不可读" << endl;                              //检查是否成功读取vec容器中的头文件信息
        return false;
    }
    uint32_t bitsPerSample = in_info.bitsPerSample;                            //输入文件采样位数
    uint32_t sampleRate = in_info.sampleRate;                                  //输入文件采样率
    uint16_t channels = in_info.channels;                                      //输入文件声道数
    int err = 0;                                                               //记录错误
    if (channels > MAX_CHANNELS) {
        cout << "channels > MAX_CHANNELS=1     声道数大于1，文件可读但不能压缩文件" << "\n" << endl;
        return false;
    }
    //*encoder记录了大小为24588字节的内存块的首地址
    OpusEncoder *encoder = opus_encoder_create(sampleRate, channels, OPUS_APPLICATION_VOIP, &err);

                                   //OPUS_APPLICATION_AUDIO     OPUS_APPLICATION_VOIP
                                   //默认定义编码器应用场景为模式二：OPUS_APPLICATION_AUDIO，在这里决定了应用模式,可以改成OPUS_APPLICATION_VOIP
    if (!encoder || err < 0) {
        fprintf(stderr, "编码失败  failed to create an encoder: %s  是出错的原因\n", opus_strerror(err));
        if (!encoder) {
            opus_encoder_destroy(encoder);
        }
        return false;
    }
    const uint16_t *data = (uint16_t *) (input->Data() + sizeof(in_info));     //指针指向文件数据流后面的音频数据
    size_t size = (input->Size() - sizeof(in_info)) / 2;                       //音频数据样本个数（每个数据两个字节）
    opus_int16 pcm_bytes[FRAME_SIZE * MAX_CHANNELS];                           //FRAME_SIZE个元素的数组，用来存放一帧的音频数据
    size_t index = 0;                                                          //记录压缩音频数据的系数
    size_t step = static_cast<size_t>(FRAME_SIZE * channels);                  // 480*channels 步长480
    FileStream encodedData;                                                    //编码后的数据流
    unsigned char cbits[MAX_PACKET_SIZE];                                      //编码后的参数数组，循环写到压缩文件中
    int a = (size / (FRAME_SIZE * channels)) + 1;//计算会压缩多少帧
    int* N = (int*)malloc(a * sizeof(int));//记录每一帧输出的参数个数
    size_t frameCount = 0;                                                     //帧计数器
    size_t readCount = 0;                                                      //压缩数据个数，用于最后一帧
    //设置初值


    while (index < size) {                                                     //pcm_bytes每次存480个文件 数据一步步处理
        //memset(&pcm_bytes, 0, sizeof(pcm_bytes));
        //memcpy(pcm_bytes, data + FRAME_SIZE*13, step * sizeof(uint16_t));
        if (index + step <= size) {                         
            memcpy(pcm_bytes, data + index, step * sizeof(uint16_t));          //将date+index中的数据传给pcm_bytes //sizeof(uint16_t)=2
            index += step;
        } else { 
            readCount = size - index;                                          //当index系数加上步长大于文件数据数量时，readcount就是剩下的文件数据数量
            memcpy(pcm_bytes, data + index, (readCount) * sizeof(uint16_t));
            index += readCount;                                                //音频数据复制的最后一步，这一步中index=size
        } 
        int nbBytes= opus_encode(encoder, pcm_bytes, channels * FRAME_SIZE, cbits, MAX_PACKET_SIZE);//一帧编码产生的码元数目，主要的编码算法函数，在这个函数中实现了opus的音频压缩
        N[frameCount] = nbBytes;//保留输出参数个数
        if (nbBytes < 0) {
            fprintf(stderr, "//编码失败nbBytes < 0//  encode failed: %s//\n", opus_strerror(nbBytes));
            break;
        }
        ++frameCount;
        //if (nbBytes > 5) nbBytes = nbBytes - 5;
        encodedData.AppendU32(static_cast<uint32_t>(nbBytes));                //将nbBytes这个数复制给encodedData
        encodedData.Append((char *) cbits, static_cast<size_t>(nbBytes));     //将编码后的数据从cbits中复制nbBytes个给encodedData
    }
    FILE* fp = fopen("nbBytes1.bin", "wb");////////////////////////////
    fwrite(N, sizeof(int),a, fp);///////////////////////////////////////////
    fclose(fp);/////////////////////////////////////////////////
    WavInfo out_info = {};
    out_info.bitsPerSample = bitsPerSample;
    out_info.sampleRate = sampleRate;
    out_info.channels = channels;
    output->SeekBeg();
    output->Append((char *) &out_info, sizeof(out_info));                     //将音频头文件信息写给压缩文件
    output->Append(encodedData.Data(), encodedData.Size());
    opus_encoder_destroy(encoder);
    return true;
}
//解压缩算法
bool Opus2Wav(FileStream *input, FileStream *output) {
    WavInfo info = {};
    input->SeekBeg();//输入的out文件读取初始化读取位置
    size_t read = input->Read(&info, sizeof(info), 1);//读取info字节的数据给info，也就是音频头文件信息
    if (read != 1) {
        return false;
    }
    int channels = info.channels;
    if (channels > MAX_CHANNELS) {
        return false;
    }
    output->SeekBeg();//输出解压缩流，初始化位置
    output->Append((char *) &info, sizeof(info));//将刚刚读取的out文件的头文件信息写入输出流中
    int err = 0;
    OpusDecoder *decoder = opus_decoder_create(info.sampleRate, channels, &err);//创建状态解码器
    if (!decoder || err < 0) {
        fprintf(stderr, "failed to create decoder: %s\n", opus_strerror(err));
        if (!decoder) {
            opus_decoder_destroy(decoder);
        }
        return false;
    }
    unsigned char cbits[MAX_PACKET_SIZE];
    opus_int16 out[MAX_FRAME_SIZE * MAX_CHANNELS];
    int frameCount = 0;
    while (true) {
        uint32_t nbBytes;
        size_t readed = input->Read(&nbBytes, sizeof(uint32_t), 1);//循环读取每一压缩帧前记录字节数的参数，赋值给nbBytes
        if (readed == 0) {
            break;
        }

        if (nbBytes > sizeof(cbits)) {//如果读取出来的数大于最大字节则编码时出现错误
            fprintf(stderr, "nbBytes > sizeof(cbits)\n");
            break;
        }
        readed = input->Read(cbits, sizeof(char), nbBytes);//将接下来nbBytes个数据读取给cbits，为了接下来每一帧的解压缩
        if (readed != nbBytes) {
            fprintf(stderr, "readed != nbBytes\n");
            break;
        }
        int frame_size = opus_decode(decoder, cbits, nbBytes, out, MAX_FRAME_SIZE, 0);//返回的frame_size实际上由解压缩cbits的第一个数确定
                                                                                      //循环解压缩每一压缩帧数据
        if (frame_size < 0) {
            fprintf(stderr, "decoder failed: %s\n", opus_strerror(frame_size));
            break;
        }
        ++frameCount;
        output->Append((char *) out, channels * frame_size * sizeof(out[0]));//将解压缩后的音频数据写入给输出流
    }
    opus_decoder_destroy(decoder);//释放空间
    return true;
}


void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}

void opus2wav(const char *in_file, char *out_file) {
    FileStream input;
    FileStream output;
    input.ReadFromFile(in_file);
    Opus2Wav(&input, &output);//解压缩
    stream2wav(&output, out_file);//将解压缩的数据转成wav格式音频，输出的wav格式默认为RIFF格式（normal wav）采用pcm格式，单声道，16采样精度，采样率和读取出来的采样率一致
}

void wav2opus(char *in_file, char *out_file) {
    FileStream input;
    FileStream output;
    wav2stream(in_file, &input);                                      //将输出音频格式的头文件信息和音频的pcm数据（2字节读取）写入到向量vec.date中
    Wav2Opus(&input, &output);                                        //压缩主要算法
    output.WriteToFile(out_file);
}

//int main(int argc, char* argv[]) {
//    if (argc < 2)//argc=2
//        return -1;
int main(){
    int argc;
    char* argv[2] = { 0 };
    //压缩文件地址
    argv[0] = "D:/VScodeproj/opus - master/out/build/x64 - Debug/opus.exe";////产生的exe运行程序
    argv[1] = "D:/VScodeproj/test01_16A.wav";//"D:/Personal/opus语音压缩/标准语音库/男声 - 快速.wav";//////压缩的音频文件
    //解压缩文件地址
    //argv[0] = "D:/VScodeproj/opus - master/out/build/x64 - Debug/opus.exe";
    //argv[1] = "D:/VScodeproj/test01_16A.out";

    char* in_file = argv[1]; //命令行参数argv[1]记录的是文件路径
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
                                   //压缩计时数据定义
    system_clock::time_point t5, t6;
    duration<double> time_span;
    chrono::milliseconds ms;

                                  /*解压缩计时数据定义*/
    clock_t s, e;
    s = clock();

    splitpath(in_file, drive, dir, fname, ext);//ext路径分解，为了接下来的.wav文件辨别，同时为了在同路径下产生输出文件
   
    if (memcmp(".wav", ext, strlen(ext)) == 0) {//判断是否为.wav文件
        sprintf(out_file, "%s%s%s.out", drive, dir, fname);//在同路径下生成输出文件
        //
        t5 = system_clock::now();//压缩计时，开始时刻
        wav2opus(in_file, out_file);//核心压缩算法
        t6 = system_clock::now();//压缩计时，结束时刻
        time_span = duration_cast<duration<double>>(t6 - t5);
        ms = duration_cast<chrono::milliseconds>(t6 - t5);
        cout << "压缩花费时间T =  " << time_span.count()*1000 << " ms.\n" << endl;
        //

    } else if (memcmp(".out", ext, strlen(ext)) == 0) {//判断是否为.out文件
        sprintf(out_file, "%s%s%s_out.wav", drive, dir, fname);//在同路径下产生解压缩文件.wav
        //
        s = clock();//解压缩计时，开始时刻
        opus2wav(in_file, out_file);//解压缩算法
        e = clock();//解压缩计时，结束时刻
        //
        cout << "解压缩花费时间T = " << (1000 * double(e - s) / CLOCKS_PER_SEC) << "ms.\n";
    }
   
    printf("done.\n");
    printf("press any key to exit.\n");
    getchar();
    return 0;
}
