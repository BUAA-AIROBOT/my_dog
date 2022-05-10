import speech_recognition as sr

class AudioModule():
    '''google api'''
    def __init__(self,wav_file_name=None,text_file_name=None,dumped_wav_file_name=None,mode="listen_mode",language="zh-CN"):
        self.recognizer = sr.Recognizer()
        self.wav_file_name = wav_file_name
        self.text_file_name = text_file_name
        self.dumped_wav_file_name = dumped_wav_file_name
        self.mode = mode
        self.language = language

    def listen(self):
        '''实时录音 并返回音频信息'''
        with sr.Microphone() as source:
            print("正在实时录音...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)
            return audio

    def read_wav(self):
        '''从本地音频文件读取音频信息'''
        if self.wav_file_name:
            print("正在读取本地wav文件...")
            with sr.AudioFile(self.wav_file_name) as source:
                audio = self.recognizer.listen(source)
                return audio

    def translate(self,audio):
        '''请求谷歌翻译服务 进行翻译'''
        try:
            print("处理中...")
            text = self.recognizer.recognize_google(audio,language=self.language)
            with open(self.text_file_name,'w') as f:
                f.write(text)
                print("已将识别结果保存在本地！")
        except sr.UnknownValueError:
            print("不能识别音频内容！")
        except sr.RequestError as e:
            print("无法请求服务！" + format(e))

    def dump(self,audio):
        '''将audio对象dump为wav文件'''
        with open(self.dumped_wav_file_name, "wb") as f:
            f.write(audio.get_wav_data(convert_rate=16000))
            print("已将音频信息保存为本地wav文件！")

    def run(self,need_dump=False):
        '''运行'''
        if self.mode == "listen_mode":
            audio = self.listen()
        else:
            audio = self.read_wav()
        self.translate(audio)
        if need_dump:
            #dump audio对象
            self.dump(audio)

if __name__ == '__main__':
    audio_module = AudioModule(wav_file_name="test_zh.wav",text_file_name="test.txt",mode="read_mode",language="zh-CN")
    audio_module.run()
