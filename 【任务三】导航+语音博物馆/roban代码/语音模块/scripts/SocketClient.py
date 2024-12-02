#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
import sys
import socket
import json
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import time

#### socket connect info ####
SERVER_IP = "127.0.0.1"
SERVER_PORT = 12000
SOCKET_CONNECT_TIMEOUT = 10

def dict_to_bytes(dict_msg:dict):
    return json.dumps(dict_msg).encode("utf-8")

class Socket_runner_client(object):
    def __init__(self):
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s_time = time.time()
        while not rospy.is_shutdown():
            try:
                self.socket_client.connect((SERVER_IP, SERVER_PORT))
                print("socket_runner_client Connected successfully")
            except Exception as err:
                rospy.logwarn("Socket_runner_client connect fail {} {}".format(err,__file__))
                rospy.sleep(1)
                if time.time() - s_time > SOCKET_CONNECT_TIMEOUT:
                    raise err
            else:
                break

    def get_aborting_status(self, req = None):
        """获取案例执行状态"""
        try:
            self.send({"cmd":"get_execution_status"})
            recdata=self.socket_client.recv(1024)
            tmp_json = json.loads(recdata.decode("utf-8"))
            return tmp_json["in_aborting"]
        except Exception as e:
            rospy.logerr("{}".format(e))
            return True
        
    def get_execution_status(self, req = None):
        """获取案例执行状态"""
        try:
            self.send({"cmd":"get_execution_status"})
            recdata=self.socket_client.recv(1024)
            tmp_json = json.loads(recdata.decode("utf-8"))
            return tmp_json["in_execution"]
        except Exception as e:
            rospy.logerr("{}".format(e))
            return True

    def send(self,msg):
        self.socket_client.send(dict_to_bytes(msg))

    def run(self,demo_path:str):
        run_res=False
        if self.get_execution_status()==False:
            msg={
                "cmd":"run_node",
                "path":demo_path
            }
            print("SocketClient is now running path program~!")
            try:
                self.send(msg)
            except Exception as e:
                print(e,__file__)
            else:
                run_res=True
        return run_res
    
    def stop(self):
        res=False
        if self.get_execution_status()==True:
            msg={
                "cmd":"stop_node",
            }
            try:
                self.send(msg)
            except Exception as e:
                print(e,__file__)
            else:
                res=True
        return res

    def close(self):
        self.socket_client.close()

if __name__ == "__main__":
    runner=Socket_runner_client()
    runner.run("/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/舵机扫描.py")
    import time
    time.sleep(3)
    runner.stop()
