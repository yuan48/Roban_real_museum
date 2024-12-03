#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import sys
sys.path.append('/home/lemon/robot_ros_application/catkin_ws/src/keyboards/scripts')
from play_tts_file import Player
from sensor_msgs.msg import BatteryState
from std_msgs.msg import *

LOW_POWER = 0
FULL_POWER = 100
PRE_BATTERY_LEVEL_PRECENT = 5
MAINBOARD_ID = 200
REGISTER_PRESENT_VOLTAGE_ADRESS = 50
VOICE_FOLDER_PATH = '/home/lemon/robot_ros_application/catkin_ws/src/keyboards/voice/'
BATTERYLEVELPRECENT = {0:10.992, 5:11.0491, 10:11.0776, 15:11.1036, 20:11.1197, 25:11.1408, 30:11.1681, 35:11.214, 40:11.2549, 45:11.3219, 50:11.3777, 55:11.4645, 60:11.5352, 65:11.6307, 70:11.7001, 75:11.7858, 80:11.9208, 85:12.0256, 90:12.1341, 95:12.2543, 100:12.4603}

class DetectBatteryLevel(Player):

    def __init__(self):
        Player.__init__(self)
        self.battery_level_precent_table = list(BATTERYLEVELPRECENT.values())
        self.battery_level_precent_table.sort()
        self.ADC_detect_data = 0
        self.ADC_detect()

    def ADC_detect(self):
        try:
            rospy.Subscriber('/MediumSize/SensorHub/BatteryState', BatteryState, self.ADC_data_callback, queue_size=1)
        except:
            print("get '/MediumSize/SensorHub/BatteryState' timed out!")

    def ADC_data_callback(self, msg):
        self.ADC_detect_data = msg.voltage

    def baseboard_voltage_transform(self, voltage):
        if voltage < self.battery_level_precent_table[0]:
            return LOW_POWER

        for index, value in enumerate(self.battery_level_precent_table):
            if voltage < value:
                return index * PRE_BATTERY_LEVEL_PRECENT - PRE_BATTERY_LEVEL_PRECENT * (value - voltage) / (value - self.battery_level_precent_table[index-1])
        return FULL_POWER

    def play_voltage_voice(self, precent):    # 0.0 ~ 100.0
        self.play_complete_voice(VOICE_FOLDER_PATH, 'voltage_precent')
        precent = str(precent)
        if precent == '100.0':
            self.play_complete_voice(VOICE_FOLDER_PATH, '100')    # hundred
        elif len(precent) == 3:
            self.play_single_voice(VOICE_FOLDER_PATH, precent[0])     # unit
        else:
            if precent[0] != '1':
                self.play_single_voice(VOICE_FOLDER_PATH, precent[0])     # Ten
            self.play_complete_voice(VOICE_FOLDER_PATH, '10')     # ten
            if precent[1] != '0':
                self.play_single_voice(VOICE_FOLDER_PATH, precent[1])     # unit
        self.play_complete_voice(VOICE_FOLDER_PATH, 'dot')      # point
        self.play_single_voice(VOICE_FOLDER_PATH, precent[-1])        # decimal

    def battery_level_detect(self):
        self.play_complete_voice(VOICE_FOLDER_PATH, 'detect_battery_level')
        try:
            if self.ADC_detect_data != 0:
                ADC_data = self.ADC_detect_data
                ADC_data = round(float(ADC_data), 2)
                battery_level_precent = round(self.baseboard_voltage_transform(ADC_data), 1)
                self.play_voltage_voice(battery_level_precent)
            else:
                print("no")
        except Exception as err:
            print("ADC_detect error: " + str(err))

if __name__ == "__main__":

    rospy.init_node('battery_node')
    dbl = DetectBatteryLevel()
    time.sleep(0.1)
    dbl.battery_level_detect()