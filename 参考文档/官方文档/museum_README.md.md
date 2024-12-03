# Ros Museum Docent

## 目录

[一、功能包目录结构](#功能包目录结构)

[二、配置文件结构](#配置文件结构)

[三、提取模板图片](#提取模板图片)

[四、自定义动作设计与下载](#自定义动作设计与下载)

[五、标定artag的位置](#标定artag的位置)

[六、增加文物配置](#增加文物配置)

[七、代码执行与停止](#代码执行与停止)

[八、可能遇到的问题和排查解决方法](#可能遇到的问题和排查解决方法)

### 功能包目录结构

    ros_museum_docent
    ├── config  # 配置文件目录
    │   └── config.json  # 配置文件
    ├── doc_img/  # 文档用图
    ├── launch
    │   └── artag_calibrate.launch  # 标定 artag 脚本启动
    │   └── museum_docent.launch  # 主程序启动
    ├── scripts  # 源代码
    │   ├── model_image  # 文物的图片，作为识别时的模板
    │   │   ├── Sanxingdui_left.jpg  # 模板图片
    │   │   ├── Sanxingdui_right.jpg
    │   │   └── Sanxingdui.jpg
    │   ├── bow_head.py  # 低头看向桌面的动作
    │   ├── feature_matching.py  # 特征匹配模块
    │   ├── main.py  # 主程序
    │   └── marker_tool.py  # 用于标定 artag 坐标位置的工具脚本
    ├── CMakeLists.txt
    ├── package.xml
    └── Readme.md

### 配置文件结构

- 默认的内容如下图所示：

    ![配置文件](doc_img/%E9%85%8D%E7%BD%AE%E6%96%87%E4%BB%B6.png)

    - 模板图片文件路径

        - 用于程序中特征匹配作为模板图片，以逗号分隔，下文有[提取模板图片](#提取模板图片)的方法

    - 顺序执行动作或走路

        - ```json
            "action_and_move_to_list": [
            "自定义动作或走路",
            "自定义动作或走路",
            "自定义动作或走路"
            ]
            ```

        - 如上动作文件和走路的数值放在一个数组中，程序中会从上到下依次执行数组的内容，内容之间以逗号分隔。按照原配置文件中的配置，机器人将顺序执行以下 5 项内容：（可以删除、增加、修改，但注意每项内容之间要以逗号分隔）

            - 执行自定义动作鞠躬

            - 向后退 40cm

            - 原地逆时针旋转 180°

            - 执行自定义动作鞠躬

            - 向左移动 20cm

        - 动作文件可自定义，下文有介绍[自定义动作设计与下载](#自定义动作设计与下载)，下载好自定义动作后，将动作文件的路径添加到配置文件中

        - 走路的数据由一个数组保存，数组包含三个值：[前后方向, 左右方向, 旋转角度] (右手定则)

        - 程序中会根据数组的数据，移动机器人到对应距离和角度，单位为米(m)

        - 如：

            - [0.0, 0.0, 180] 是机器人原地逆时针旋转 180°（由于零点原因，可能存在误差）

            - [-0.20, 0.0, 0.0] 是机器人向后退 20cm（由于零点原因，可能存在误差）

            - [0.0, 0.1, 0.0] 是机器人向左移动 10cm（由于零点原因，可能存在误差）

    - 介绍的文字或播放的音频

        - 介绍的内容和音频文件的路径（参照上条动作文件路径书写）存放在一个列表中，程序中会顺序执行列表的内容，以逗号分隔

        - 程序会将文字内容转成语音播放，如果是音频文件则播放音频文件，执行完一条再执下一条

    - 坐标和角度阈值

        - 机器人在校准定位时，如果当前坐标与目标坐标差值的绝对值小于对应的阈值，则认为到达目标坐标或距离

    - 目标点坐标和角度

        - 可以用[标定脚本](#标定-artag-的位置)自动修改

### 提取模板图片

- 摄像头图像获取

    可以在网页输入：

    `http://${IP}:8080/snapshot?topic=/camera/color/image_raw`

    其中 ${IP} 是机器人的 IPv4

    如机器人的 ip 为 `192.168.3.100` ，那么网页应该输入 `http://192.168.3.100:8080/snapshot?topic=/camera/color/image_raw`

    这里显示的是头部摄像头一帧的图像，可以在移动机器人后刷新网页，查看当前位置摄像头的图像

- 低头获取物品图像

    - 运行低头的动作：

        ```shell
        source ~/robot_ros_application/catkin_ws/devel/setup.bash
        rosservice call /MediumSize/BodyHub/StateJump 2 setStatus
        rosrun ros_museum_docent bow_head.py
        ```

    - 图像可参考 `/home/lemon/robot_ros_application/catkin_ws/src/ros_museum_docent/scripts/model_image` 目录下的图片进行截取

    - 右键网页中的图片，保存到本地，再将图片存放到 `/home/lemon/robot_ros_application/catkin_ws/src/ros_museum_docent/scripts/model_image` 目录下（tips: 可以针对一件文物，保存机器人在不同角度看到的图像）

    - 恢复站立:

        ```shell
        source ~/robot_ros_application/catkin_ws/devel/setup.bash
        rosservice call /MediumSize/BodyHub/StateJump 2 reset
        ```

### 自定义动作设计与下载

- 生成自定义动作方法

    - 用 Roban 软件编辑动作下载到机器人中

- Roban 软件动作设计步骤：

    - 新建工程文件，应该以每一个动作新建独立的工程文件，下载到机器人内会在指定目录生成以工程文件命名的 python 文件

        ![新建工程文件](../../../doc/AIUI配置FAQ_IMG/新建工程.png)

    - 选择插入帧的位置，右键添加关键帧，选择全身即可

        ![插入关键帧](../../../doc/AIUI配置FAQ_IMG/插入关键帧.png)

    - 点击软件界面右侧机器人模型，选择需要控制的机器人关节部位，选择需要控制的舵机，拖动时间轴或输入码盘数值，机器人相应位置舵机会发生转动

    - 再点击确认按钮，当前这一帧的动作便会插入到动作帧位置。需要注意的是，动作帧时间轴单位 1f = 10ms

        ![插入关键帧](../../../doc/AIUI配置FAQ_IMG/设置舵机码盘的数值.png)

    - 按同样的步骤，在开始帧和结束帧之间插入多个关键帧

    - 点击 "生成模块"，会将动作帧内容以模块化形式进行封装

        ![生成自定义动作](../../../doc/AIUI配置FAQ_IMG/生成自定义动作.png)

    - 将生成的动作模块，放在开始程序内，点击 "下载"

        ![下载动作](../../../doc/AIUI配置FAQ_IMG/下载动作.png)

- 文件将被下载到 `/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/` 下，以工程文件命名的 python 文件

### 标定artag的位置

- 可以使用标定脚本进行自动标定，运行方式：

    ```shell
    source ~/robot_ros_application/catkin_ws/devel/setup.bash
    roslaunch ros_museum_docent artag_calibrate.launch
    ```

    ![标定artag脚本运行](doc_img/%E6%A0%87%E5%AE%9Aartag%E8%84%9A%E6%9C%AC%E8%BF%90%E8%A1%8C.png)

    本文档测试环境：距离 artag 约 76cm ，摄像头看到的图像如下：

    ![标定artag图像](doc_img/%E6%A0%87%E5%AE%9Aartag%E5%9B%BE%E5%83%8F.jpg)

- 将机器人放于 artag 码前，敲击回车键，该脚本会将当前位置的 artag 坐标写入配置文件中

    ![标定artag坐标](doc_img/artag%E6%A0%87%E5%AE%9A.png)

    标定结束后机器人会恢复站立，或敲击 "q" 键退出标定程序

### 增加文物配置

- 默认的配置文件中只有一件三星堆文物，其中包含

    - 该文物的模板图片

    - 对该文物的介绍及音频

    - 介绍该文物时的动作

    如果增加新的文物，则必须包含以上三项内容

- 如增加一件文物兵马俑，那么配置文件应该如下：

    ```json
    "elements_and_results_of_matching": {
    "三星堆": {
    "model_image_path": [
    "xxx.jpg",
    "xxx.png"
    ],
    "action_and_move_to_list": [
    "xxx"
    ],
    "museum_introduce_text": [
    "xxxxx"
    ]
    },
    "兵马俑": {
    "model_image_path": [
    "xxx.jpg",
    "xxx.png"
    ],
    "action_and_move_to_list": [
    "xxx"
    ],
    "museum_introduce_text": [
    "xxxxx"
    ]
    }
    }
    ```

    - 每一件文物应该以逗号区分包含在 `elements_and_results_of_matching` 中。每件文物应该包含该文物的模板图片、对该文物的介绍及音频和介绍该文物时的动作，且每一类以逗号分隔

    - 注意标点符号是英文标点，可以使用中文字符

### 代码执行与停止

- 本文档的测试环境

    - 文物道具一件

    - ARTag 尺寸：10.0x10.0cm

    - ARTag 距离桌面：33.2cm

    - 距离文物道具台面约 1m 距离启动程序

- 通过 launch 文件运行 artag 节点 和 本功能包主程序节点

    ```shell
    source ~/robot_ros_application/catkin_ws/devel/setup.bash
    roslaunch ros_museum_docent museum_docent.launch
    ```

- 程序执行完毕将自动退出，机器人恢复站立

### 可能遇到的问题和排查解决方法

- 报错 `ValueError: <string>:** Unexpected ** at column *`

    - 通常是修改配置文件时格式出错，多余或者缺少标点符号，可以根据 `Unexpected` 前的数字代表行号，到配置文件中查看该行或该行的上下相邻几行是否有标点或字符的错误

- 标定 artag 时，机器人虽位于 artag 码前，但始终获取不到坐标数据，提示没有识别到 artag 码

    - 可以用网页打开头部摄像头视频流查看机器人摄像头是否正常显示且看全了 artag 码

        - `http://${IP}:8080/stream_viewer?topic=/camera/color/image_raw`

            其中 ${IP} 是机器人的 IPv4

            如果无图像显示，可以尝试手动重启机器人的所有节点

            ```shell
            sudo killall roslaunch
            bash ~/start.sh
            ```

            待终端不再往下输出后，再通过网页查看摄像头图像

        - 如果看全了 artag 码（外圈的黑色边框也不能被遮挡），则敲击回车尝试标定
