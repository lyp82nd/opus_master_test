#include <iostream>
#include <opus_types.h>
#include  <opus.h>
#include <cstring>
#include <memory>

#include <vector>
// https://github.com/mackron/dr_libs/blob/master/dr_wav.h
// 
//��ʱ����ͷ�ļ�
#include <time.h>
#include <chrono>

using namespace std;
using namespace std::chrono;
//
#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"//wav�⺯��

//ͨ��ȡֵΪ2.5ms��5ms��10ms��20ms��40ms��60ms����Ӧ�Ĳ������ֱ�Ϊ120��240��480��960��1920��2880��
//2.5ms��120�������������ڼ����ӳٵ�ʵʱͨ�ų���������������Ϸ����Ƶ����ȡ�(���лᱨ��)
//5ms��240�������������ڵ��ӳٵ�ʵʱͨ�ų���������VoIP�绰������㲥�ȡ�(���лᱨ��)
//10ms��480����������������ͨ��ʵʱͨ�ų�����������Ƶ���顢��������ȡ�
//20ms��960����������������Ҫƽ���ӳٺ���Ƶ�����ĳ������������߽�����Զ��Э���ȡ�
//40ms��1920��������������ǿ����Ƶ�����ĳ�������������ʶ�������洢�ȡ�(���лᱨ��)
//60ms��2880�������������ڸ�Ʒ����Ƶ�ĳ���������������ý�塢¼���ȡ�(���лᱨ��)
#define FRAME_SIZE 80     //֡��С�㷨=fs*������msΪһ֡�������8k�����ʣ�10msΪһ֡��֡��С����80���ٷ��ļ���˵20msΪһ֡�����з�Ϊ�ĸ���֡5ms���������������Ϊ8k��20msΪһ֡��֡��СΪ8k*20ms=160,�������������̫����
#define MAX_FRAME_SIZE (6*FRAME_SIZE)//

#define MAX_CHANNELS 1
//һ֡�������ѹ����3*1276���ֽ�
#define MAX_PACKET_SIZE (3*1276)

#pragma pack(push)
#pragma pack(1)

//���ṹ�嶨��Ϊһ���ļ���Ϣ�������������Էֱ����������Ͳ����ʺͲ���λ��
struct WavInfo {
    uint16_t channels;//������
    uint32_t sampleRate;//������
    uint32_t bitsPerSample;//����λ��
};

#pragma pack(pop)
//���ָ��û�б������������Ϊ��ָ��
#ifndef  nullptr
#define  nullptr NULL
#endif

class FileStream {
public:
    FileStream() {//�޲ι��캯������ʼ�� cur_pos = 0�Ǽ�¼��ǰλ�õĲ���Ϊ0
        cur_pos = 0;
    }
    //Append���壬���ã���date���渴��size���ֽڵ�vec.data() + cur_pos�У�ͬʱcur_pos������size+cur_posλ��
    void Append(const char *data, size_t size) {
        if (cur_pos + size > Size()) {
            vec.resize(cur_pos + size);
        }
        memcpy(vec.data() + cur_pos, data, size);//��date���渴��size���ֽڵ�vec.data() + cur_pos��
        cur_pos += size;
    }

    void AppendU32(uint32_t val) {
        Append((char *) (&val), sizeof(val));
    }

    char *Data() {
        return vec.data();
    }
//�������������������
    size_t Size() {
        return vec.size();
    }

    //���ļ���Ϣ��vec������ǰelemCount*elemSize��λ���ж���buff�У����ҷ���read=1��cur_pos=cur_pos+elemCount
    size_t Read(void *buff, size_t elemSize, size_t elemCount) {
        size_t readed = std::min((vec.size() - cur_pos), (elemCount * elemSize)) / elemSize;
        if (readed > 0) {
            memcpy(buff, vec.data() + cur_pos, readed * elemSize);
                    //��vec.data() + cur_pos�е����ݸ���readed * elemSize����buff
            cur_pos += readed * elemSize; 
                    //cur_pos����cur_pos=cur_pos+readed * elemSize��cur_pos=cur_pos+10��
        }
        return readed;
    }

    //������vec�ǿյ�cur_pos=0���粻��cur_pos=��vec��������-1����ֵ��������cur_pos=0
    bool SeekCur(int offset) {
        //offset=0��cur_pos=0
        if (cur_pos + offset > vec.size()) {
            cur_pos = !vec.empty() ? (vec.size() - 1) : 0;//vec.empty()�����ж������Ƿ�Ϊ�յ�
            return false;                                 //vec.size()������vec��������
        } else {
            cur_pos += offset;
            return true;
        }
    }
    //���û�в���������cur_pos = 0
    bool SeekBeg(int offset = 0) {
        cur_pos = 0;
        return SeekCur(offset);
    }

    bool WriteToFile(const char *filename) {
        FILE *fin = fopen(filename, "wb");//��������ļ���ָ��������ļ�ָ�뱻���أ��������ɹ�����NULL 
        if (!fin) {
            return false;
        }
        fseek(fin, 0, SEEK_SET);//ָ���ļ���ʼλ��
        fwrite(vec.data(), sizeof(char), vec.size(), fin);//�洢��vec�����е���������д�뵽�ļ��У�����ζ��vec�д�������յ�ѹ��������
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
    std::vector<char> vec;//�����Ķ���
    size_t cur_pos;
};

bool Wav2Opus(FileStream *input, FileStream *output);

bool Opus2Wav(FileStream *input, FileStream *output);

bool wav2stream(char *input, FileStream *output);

bool stream2wav(FileStream *input, char *output);

//�����wav��ʽĬ��Ϊriff��ʽ��normal wav��
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
//����һ��wav���������׵�ַ��*wavBuffer��¼�������ݵ��׵�ַ,
int16_t *wavRead_int16(char *filename, uint32_t *sampleRate, uint64_t *totalSampleCount) {
    unsigned int channels;//ͨ����
    int16_t *buffer = drwav_open_and_read_file_s16(filename, &channels, sampleRate, totalSampleCount);
    if (buffer == nullptr) {
        fprintf(stderr, "wav��Ƶ��ȡʧ��ERROR\n");
        return nullptr;
    }
    if (channels != 1) {
        drwav_free(buffer);
        buffer = nullptr;
        *sampleRate = 0;
        *totalSampleCount = 0;
        cout << "���ǵ�������Ƶopus.cpp179 \n" << endl;
    }
    return buffer;//���ص��ǵ�ַ
}

bool wav2stream(char *input, FileStream *output) {
    uint32_t sampleRate = 0;//�����ʶ���Ϊ0
    uint64_t totalSampleCount = 0;//����������������ֽ�Ϊ��λ
    int16_t *wavBuffer = wavRead_int16(input, &sampleRate, &totalSampleCount);//����һ��int16��ȡ��wav��������*wavBuffer��¼���ǵ�ַ
    if (wavBuffer == nullptr) return false;
    WavInfo info = {};

    //ǿ�ƵĶ����������Ƶ�ĸ�ʽ
    info.bitsPerSample = 16;
    info.sampleRate = sampleRate;//������������һ��
    info.channels = 1;
    cout << "�����Ƶ�ļ���ʽ(����λ���̶�16λ��������������һ�£�ǿ�������ƵΪ��������Ƶ)" << "\n" << endl;
    output->SeekBeg();
    output->Append((char *) &info, sizeof(info));//��info��Ϣ����д�뵽vec.date��
    output->Append((char *) wavBuffer, totalSampleCount * sizeof(int16_t));//��wav���ݴ�wavbuffָ����ָ��λ��д������vec.date�У�����wavbufferǿ��ת��char����totalSampleCount��Ҫ��ÿ��ռ���ֽڵ����ݷֳ�����������������ȷд��
    //totalSampleCount * sizeof(int16_t)=112640,cur_pos=10(ͷ�ļ���Ϣ��+112640��wav���ݣ����ֽڵ�����56320��һ��112640���ֽڵ����ݣ�
    free(wavBuffer);
    return true;
}

bool stream2wav(FileStream *input, char *output) {//input�����ǽ�ѹ���ı�������output��������������Ƶ
    WavInfo info = {};
    input->SeekBeg();
    size_t read = input->Read(&info, sizeof(info), 1);//��ȡ��ѹ�����ͷ�ļ���Ϣ
    if (read != 1) {
        return false;
    }
    size_t totalSampleCount = (input->Size() - sizeof(info)) / 2;
    return wavWrite_int16(output, (int16_t *) (input->Data() + sizeof(info)), info.sampleRate,
                          static_cast<uint32_t>(totalSampleCount));                //�����wav��ʽĬ��ΪRIFF��ʽ��normal wav������pcm��ʽ����������16�������ȣ������ʺͶ�ȡ�����Ĳ�����һ��
}

bool Wav2Opus(FileStream *input, FileStream *output) {
                         
    WavInfo in_info = {};
    input->SeekBeg();
    size_t read = input->Read(&in_info, sizeof(in_info), 1);                   //��vec�����ж�ȡͷ�ļ���Ϣд���ṹ��in_info
    if (read != 1) {
        cout << "read!=1     �ļ����ɶ�" << endl;                              //����Ƿ�ɹ���ȡvec�����е�ͷ�ļ���Ϣ
        return false;
    }
    uint32_t bitsPerSample = in_info.bitsPerSample;                            //�����ļ�����λ��
    uint32_t sampleRate = in_info.sampleRate;                                  //�����ļ�������
    uint16_t channels = in_info.channels;                                      //�����ļ�������
    int err = 0;                                                               //��¼����
    if (channels > MAX_CHANNELS) {
        cout << "channels > MAX_CHANNELS=1     ����������1���ļ��ɶ�������ѹ���ļ�" << "\n" << endl;
        return false;
    }
    //*encoder��¼�˴�СΪ24588�ֽڵ��ڴ����׵�ַ
    OpusEncoder *encoder = opus_encoder_create(sampleRate, channels, OPUS_APPLICATION_VOIP, &err);

                                   //OPUS_APPLICATION_AUDIO     OPUS_APPLICATION_VOIP
                                   //Ĭ�϶��������Ӧ�ó���Ϊģʽ����OPUS_APPLICATION_AUDIO�������������Ӧ��ģʽ,���Ըĳ�OPUS_APPLICATION_VOIP
    if (!encoder || err < 0) {
        fprintf(stderr, "����ʧ��  failed to create an encoder: %s  �ǳ����ԭ��\n", opus_strerror(err));
        if (!encoder) {
            opus_encoder_destroy(encoder);
        }
        return false;
    }
    const uint16_t *data = (uint16_t *) (input->Data() + sizeof(in_info));     //ָ��ָ���ļ��������������Ƶ����
    size_t size = (input->Size() - sizeof(in_info)) / 2;                       //��Ƶ��������������ÿ�����������ֽڣ�
    opus_int16 pcm_bytes[FRAME_SIZE * MAX_CHANNELS];                           //FRAME_SIZE��Ԫ�ص����飬�������һ֡����Ƶ����
    size_t index = 0;                                                          //��¼ѹ����Ƶ���ݵ�ϵ��
    size_t step = static_cast<size_t>(FRAME_SIZE * channels);                  // 480*channels ����480
    FileStream encodedData;                                                    //������������
    unsigned char cbits[MAX_PACKET_SIZE];                                      //�����Ĳ������飬ѭ��д��ѹ���ļ���
    int a = (size / (FRAME_SIZE * channels)) + 1;//�����ѹ������֡
    int* N = (int*)malloc(a * sizeof(int));//��¼ÿһ֡����Ĳ�������
    size_t frameCount = 0;                                                     //֡������
    size_t readCount = 0;                                                      //ѹ�����ݸ������������һ֡
    //���ó�ֵ


    while (index < size) {                                                     //pcm_bytesÿ�δ�480���ļ� ����һ��������
        //memset(&pcm_bytes, 0, sizeof(pcm_bytes));
        //memcpy(pcm_bytes, data + FRAME_SIZE*13, step * sizeof(uint16_t));
        if (index + step <= size) {                         
            memcpy(pcm_bytes, data + index, step * sizeof(uint16_t));          //��date+index�е����ݴ���pcm_bytes //sizeof(uint16_t)=2
            index += step;
        } else { 
            readCount = size - index;                                          //��indexϵ�����ϲ��������ļ���������ʱ��readcount����ʣ�µ��ļ���������
            memcpy(pcm_bytes, data + index, (readCount) * sizeof(uint16_t));
            index += readCount;                                                //��Ƶ���ݸ��Ƶ����һ������һ����index=size
        } 
        int nbBytes= opus_encode(encoder, pcm_bytes, channels * FRAME_SIZE, cbits, MAX_PACKET_SIZE);//һ֡�����������Ԫ��Ŀ����Ҫ�ı����㷨�����������������ʵ����opus����Ƶѹ��
        N[frameCount] = nbBytes;//���������������
        if (nbBytes < 0) {
            fprintf(stderr, "//����ʧ��nbBytes < 0//  encode failed: %s//\n", opus_strerror(nbBytes));
            break;
        }
        ++frameCount;
        //if (nbBytes > 5) nbBytes = nbBytes - 5;
        encodedData.AppendU32(static_cast<uint32_t>(nbBytes));                //��nbBytes��������Ƹ�encodedData
        encodedData.Append((char *) cbits, static_cast<size_t>(nbBytes));     //�����������ݴ�cbits�и���nbBytes����encodedData
    }
    FILE* fp = fopen("nbBytes1.bin", "wb");////////////////////////////
    fwrite(N, sizeof(int),a, fp);///////////////////////////////////////////
    fclose(fp);/////////////////////////////////////////////////
    WavInfo out_info = {};
    out_info.bitsPerSample = bitsPerSample;
    out_info.sampleRate = sampleRate;
    out_info.channels = channels;
    output->SeekBeg();
    output->Append((char *) &out_info, sizeof(out_info));                     //����Ƶͷ�ļ���Ϣд��ѹ���ļ�
    output->Append(encodedData.Data(), encodedData.Size());
    opus_encoder_destroy(encoder);
    return true;
}
//��ѹ���㷨
bool Opus2Wav(FileStream *input, FileStream *output) {
    WavInfo info = {};
    input->SeekBeg();//�����out�ļ���ȡ��ʼ����ȡλ��
    size_t read = input->Read(&info, sizeof(info), 1);//��ȡinfo�ֽڵ����ݸ�info��Ҳ������Ƶͷ�ļ���Ϣ
    if (read != 1) {
        return false;
    }
    int channels = info.channels;
    if (channels > MAX_CHANNELS) {
        return false;
    }
    output->SeekBeg();//�����ѹ��������ʼ��λ��
    output->Append((char *) &info, sizeof(info));//���ոն�ȡ��out�ļ���ͷ�ļ���Ϣд���������
    int err = 0;
    OpusDecoder *decoder = opus_decoder_create(info.sampleRate, channels, &err);//����״̬������
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
        size_t readed = input->Read(&nbBytes, sizeof(uint32_t), 1);//ѭ����ȡÿһѹ��֡ǰ��¼�ֽ����Ĳ�������ֵ��nbBytes
        if (readed == 0) {
            break;
        }

        if (nbBytes > sizeof(cbits)) {//�����ȡ����������������ֽ������ʱ���ִ���
            fprintf(stderr, "nbBytes > sizeof(cbits)\n");
            break;
        }
        readed = input->Read(cbits, sizeof(char), nbBytes);//��������nbBytes�����ݶ�ȡ��cbits��Ϊ�˽�����ÿһ֡�Ľ�ѹ��
        if (readed != nbBytes) {
            fprintf(stderr, "readed != nbBytes\n");
            break;
        }
        int frame_size = opus_decode(decoder, cbits, nbBytes, out, MAX_FRAME_SIZE, 0);//���ص�frame_sizeʵ�����ɽ�ѹ��cbits�ĵ�һ����ȷ��
                                                                                      //ѭ����ѹ��ÿһѹ��֡����
        if (frame_size < 0) {
            fprintf(stderr, "decoder failed: %s\n", opus_strerror(frame_size));
            break;
        }
        ++frameCount;
        output->Append((char *) out, channels * frame_size * sizeof(out[0]));//����ѹ�������Ƶ����д��������
    }
    opus_decoder_destroy(decoder);//�ͷſռ�
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
    Opus2Wav(&input, &output);//��ѹ��
    stream2wav(&output, out_file);//����ѹ��������ת��wav��ʽ��Ƶ�������wav��ʽĬ��ΪRIFF��ʽ��normal wav������pcm��ʽ����������16�������ȣ������ʺͶ�ȡ�����Ĳ�����һ��
}

void wav2opus(char *in_file, char *out_file) {
    FileStream input;
    FileStream output;
    wav2stream(in_file, &input);                                      //�������Ƶ��ʽ��ͷ�ļ���Ϣ����Ƶ��pcm���ݣ�2�ֽڶ�ȡ��д�뵽����vec.date��
    Wav2Opus(&input, &output);                                        //ѹ����Ҫ�㷨
    output.WriteToFile(out_file);
}

//int main(int argc, char* argv[]) {
//    if (argc < 2)//argc=2
//        return -1;
int main(){
    int argc;
    char* argv[2] = { 0 };
    //ѹ���ļ���ַ
    argv[0] = "D:/VScodeproj/opus - master/out/build/x64 - Debug/opus.exe";////������exe���г���
    argv[1] = "D:/VScodeproj/test01_16A.wav";//"D:/Personal/opus����ѹ��/��׼������/���� - ����.wav";//////ѹ������Ƶ�ļ�
    //��ѹ���ļ���ַ
    //argv[0] = "D:/VScodeproj/opus - master/out/build/x64 - Debug/opus.exe";
    //argv[1] = "D:/VScodeproj/test01_16A.out";

    char* in_file = argv[1]; //�����в���argv[1]��¼�����ļ�·��
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
                                   //ѹ����ʱ���ݶ���
    system_clock::time_point t5, t6;
    duration<double> time_span;
    chrono::milliseconds ms;

                                  /*��ѹ����ʱ���ݶ���*/
    clock_t s, e;
    s = clock();

    splitpath(in_file, drive, dir, fname, ext);//ext·���ֽ⣬Ϊ�˽�������.wav�ļ����ͬʱΪ����ͬ·���²�������ļ�
   
    if (memcmp(".wav", ext, strlen(ext)) == 0) {//�ж��Ƿ�Ϊ.wav�ļ�
        sprintf(out_file, "%s%s%s.out", drive, dir, fname);//��ͬ·������������ļ�
        //
        t5 = system_clock::now();//ѹ����ʱ����ʼʱ��
        wav2opus(in_file, out_file);//����ѹ���㷨
        t6 = system_clock::now();//ѹ����ʱ������ʱ��
        time_span = duration_cast<duration<double>>(t6 - t5);
        ms = duration_cast<chrono::milliseconds>(t6 - t5);
        cout << "ѹ������ʱ��T =  " << time_span.count()*1000 << " ms.\n" << endl;
        //

    } else if (memcmp(".out", ext, strlen(ext)) == 0) {//�ж��Ƿ�Ϊ.out�ļ�
        sprintf(out_file, "%s%s%s_out.wav", drive, dir, fname);//��ͬ·���²�����ѹ���ļ�.wav
        //
        s = clock();//��ѹ����ʱ����ʼʱ��
        opus2wav(in_file, out_file);//��ѹ���㷨
        e = clock();//��ѹ����ʱ������ʱ��
        //
        cout << "��ѹ������ʱ��T = " << (1000 * double(e - s) / CLOCKS_PER_SEC) << "ms.\n";
    }
   
    printf("done.\n");
    printf("press any key to exit.\n");
    getchar();
    return 0;
}
