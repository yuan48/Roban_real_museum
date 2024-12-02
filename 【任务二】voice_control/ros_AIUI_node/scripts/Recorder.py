#!/usr/bin python
# coding=utf-8
import pyaudio

STREAM_AUDIO_TOPIC = "/audio/stream"

class Recorder:
    def __init__(self, chunk, channels=1, rate=16000, format=pyaudio.paInt16):
        self.chunk = chunk
        self.channels = channels
        self.rate = rate
        self.format = format
        self.audio = pyaudio.PyAudio()
        self.device_index = None

        count = self.audio.get_device_count()
        for i in range(count):
            device = self.audio.get_device_info_by_index(i)
            if "sysdefault" == device["name"]:
                self.device_index = i if device["maxInputChannels"] > 0 else None
                break

    def read(self):
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk
        )
        try:
            while True:
                yield stream.read(self.chunk)
        finally:
            stream.stop_stream()
            stream.close()

if __name__ == "__main__":
    import time
    chunk = 640
    recorder = Recorder(chunk=chunk)
    start_time = time.time()
    recording_time = 5
    for data in recorder.read():
        if time.time() - start_time > 5:
            break
        print("update record data")
