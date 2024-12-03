# Botec比赛程序接口说明

## 话题

#### 订阅

- /gaitCommand ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
  行走模式下用于机器人行走控制的脚步。

- /chin/visualization_marker_chin (visualization_msgs/Marker)

  下巴摄像头读取ar_tag码

- /head/visualization_marker_head (visualization_msgs/Marker)

  头部摄像头读取ar_tag码

- /initialpose (geometry_msgs/PoseWithCovarianceStamped)

  获取slam地图坐标数据

- /chin_camare/image (sensor_msgs/Image)

  下巴摄像头数据

- /camera/color/image_raw (sensor_msgs/Image)

  头部摄像头数据

- /camera/detpth/image_rect_raw (sensor_msgs/Image)

  深度摄像头数据

#### 发布

- /requestGaitCommand ([std_msgs/Bool][std_msgs/Bool])  
  行走模式下，机器人中脚步指令低于一定值时，请求新的控制脚步。

## 模块

- bodyhub_client

  - walk ()

    设置状态为"walk"。

    检查controlID是否为0或1，不是则返回False；获取状态参数为空则返回-1，参数为"walking"则返回True；如果重置状态参数失败则返回False，否则将状态参数设置为"walking"，失败则返回-2，如果状态参数为28则返回True

    ```
    @return True: "walking"
    		False: controlID不为0或1，或设置状态为"walking"失败
    		-1: 状态参数为空
    		-2: 状态参数设置为"walking"失败
    ```

  - wlaking (delta_x, delta_y, theta)

    走一步，如果超过舵机最大限幅则会卡住。

    ```
    @param delta_x: 前后距离，前为正，后为负
    	   delta_y: 左右距离，左为正，右为负
    	   theta: 旋转角度，左为正，右为负
    ```

  - wait_walking_done ()

    等待行走结束。

    循环检测walking状态，walking结束即退出

  - walking_the_distance (delta_x, delta_y, theta)

    走一段固定距离。

    ```
    @param delta_x: 前进距离
    	   delta_y: 左右距离
    	   theta: 旋转角度
    ```

  - walking_n_steps (cmd, n)

    按照固定步幅走n步。

    ```
    @param cmd: 步幅，与 walking() 的参数一致
    	   n: 步数
    ```

- bodyhub_action

  - Movement.linearMove (keyframes)

    设置(多个)舵机角度。

    ```
    @param keyframes: 设置相对当前机器人舵机角度转动的角度
    ```

  - bodyhub_ready ()

    当状态跳转至ready失败则输出log。

  - bodyhub_walk ()

    当状态跳转至walking失败则输出log。
    
  - set_arm_mode (mode)

    设置手的模式。

    ```
    @param mode: "0"或"1"，"0":动作模式，"1":步态模式
    ```

- client_action

  - custom_action (music, act_frames)

    定制动作。

    ```
    @param music: 音乐文件
    	   act_framse: 动作帧
    ```

- Navigation

  - set_debug_print (_debug_mask)

    打印当前点（slam或ar_tag坐标）。

    ```
    @param _debug_mask: 从对应的消息读取坐标
    ```

  - path_tracking (path_point, mode=1)

    设置slam引导的路径。

    ```
    @param path_point: 路径点
    ```

    

