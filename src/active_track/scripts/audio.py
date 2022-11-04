from vosk import Model, KaldiRecognizer
import os
import wave
import queue
import sounddevice as sd
import sys

class AudioModule():
    """
    语音识别模块:处理流程，先录制然后将音频信息作为wav文件保存在本地，然后读取wav文件进行语音识别，最后将语音信息作为txt文件保存在本地
    dump_file_name可以指定读取的wav文件,默认tmp.wav
    text_file_name可以指定txt文件的名字，默认tmp.txt
    mode: read_mode:将指定的wav文件读入并进行语音识别,完成后将结果dump到本地
          listen_mode:利用阻塞队列实现实时语音识别
    run:调用run方法即可运行
    """
    def __init__(self,dump_file_name=None,text_file_name=None,mode="read_mode"):
        self.dump_file_name = dump_file_name
        self.text_file_name = text_file_name
        self.mode = mode
        self.dir = "audio"
        if not os.path.exists(self.dir):
            os.mkdir(self.dir)
        self.load_model()
        self.q = queue.Queue()

    def listen(self):
        '''实时录音 并返回音频信息'''
        def callback(in_data, frames, time, status):
            '''回调函数'''
            if status:
                print(status, file=sys.stderr)
            self.q.put(bytes(in_data))

        with sd.RawInputStream(samplerate=16000, blocksize=8000, device=None, dtype='int16',
                               channels=1, callback=callback):
            print('#' * 80)
            print('请说"结束"以结束语音识别程序!')
            print('#' * 80)
            rec = KaldiRecognizer(self.model, 16000)
            while True:
                data = self.q.get()
                if rec.AcceptWaveform(data):
                    s = eval(rec.Result())["text"].replace(' ','')
                    if s == "结束":
                        break
                    else:
                        print(s)
                else:
                    rec.PartialResult()

    def read_wav(self):
        '''从本地音频文件读取音频信息 返回Wave_read文件对象'''
        try:
            print("正在读取本地wav文件...")
            file_name = self.dump_file_name
            wf = wave.open(os.path.join(self.dir,file_name), "rb")
            if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
                print("必须是WAV文件(mono PCM)!")
                exit(1)
            return wf
        except FileNotFoundError:
            print("文件不存在")
            exit(1)

    def load_model(self):
        '''载入模型'''
        print("开始载入模型!")
        self.model = Model("model")
        print("载入模型完毕！")

    def recognize(self,wf):
        '''请求识别服务 进行识别'''
        assert wf
        rec = KaldiRecognizer(self.model, wf.getframerate())
        rec.SetWords(True)
        while True:
            data = wf.readframes(4000)
            #结束
            if len(data) == 0:
                break
            rec.AcceptWaveform(data)
        #str -> dict
        res_dict = eval(rec.FinalResult())
        text = res_dict["text"]
        #过滤停顿带来的空格
        text = text.replace(' ','')
        file_name = self.text_file_name if self.text_file_name else "tmp.txt"
        print(text)
        # with open(file_name,'w') as f:
        #     f.write(text)
        #     print("已将识别结果保存在本地！")

    def run(self):
        '''运行'''
        if self.mode == "listen_mode":
            self.listen()
        else:
            self.recognize(self.read_wav())

if __name__ == '__main__':
    audio_module = AudioModule(dump_file_name="test_zh2.wav", text_file_name="test.txt",mode="listen_mode")
    audio_module.run()
