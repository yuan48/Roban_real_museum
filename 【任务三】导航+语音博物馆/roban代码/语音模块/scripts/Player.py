# coding=utf-8
import os
from threading import Thread
import rospy
from std_msgs.msg import Empty
import time
TTS_SAVE_PCM = "/tmp/aiui_voice.pcm"

class Player:
    def __init__(self):
        self.__is_playing = False
        self.__is_abort = False
        self.current_url = ""
        self.play_end = rospy.Publisher("/aiui/play_end", Empty, queue_size=1)
        self.stop_play = rospy.Subscriber("/aiui/stop_play", Empty, self.stop_play_callback, queue_size=1)

    def stop_play_callback(self, msg):
        self.stop()

    @property
    def is_playing(self):
        return self.__is_playing

    @property
    def is_abort(self):
        return self.__is_abort

    def __play(self):
        self.__is_abort = False
        self.__is_playing = True
        os.system("ffplay -f s16le -ar 16000 -ac 1 {} -nodisp -loglevel quiet -autoexit".format(self.current_url))
        self.__is_playing = False
        self.play_end.publish()
        self.remove_tts_save_pcm()

    def play(self, url):
        if self.__is_playing == True:
            self.stop()
        self.current_url = url
        Thread(target=self.__play).start()

    def wait_for_stop(self,timeout=20):
        stime = time.time()
        while self.__is_playing == True and time.time() - stime < timeout:
            time.sleep(0.01)
        
    def stop(self):
        if self.__is_playing == True:
            self.__is_playing = False
            os.system("ps -auxww | grep {} | awk '{{print $2}}' | xargs kill".format(self.current_url))
            self.remove_tts_save_pcm()
            self.__is_abort = True

    def remove_tts_save_pcm(self):
        try:
            if os.path.exists(TTS_SAVE_PCM) == True:
                os.remove(TTS_SAVE_PCM)
        except Exception as e:
            rospy.logwarn(e)

if __name__ == "__main__":
    player = Player()
    player.play(
        "/home/lemon/Music/text_to_speech.wav")
    import time
    time.sleep(1)
    player.stop()
    print(player.is_playing)

