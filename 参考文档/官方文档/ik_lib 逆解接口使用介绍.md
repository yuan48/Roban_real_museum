## ik_lib 逆解接口使用介绍

为了方便控制机器人的动作执行，我们封装了一个方便的逆解库，提供了python的逆解接口，通过该接口给逆解模块发送末端位置即可自动解算出各个关节的角度并确保机器人的末端轨迹。

- **ikmodulesim.py**

class IkModuleSim(object):

```python
def toInitPoses(self):
      #机器人位姿初始化，在动作执行前调用
      #初始化成功 返回 True ,否则，返回 False
```

```python
def body_motion(self, body_pose, value, count=100):
    # 机器人末端动作执行，给定目标位姿 body_pose，value 通过逆解得到舵机角度，控制机器人运动
    Args:
        - body_pose: CtrlType, or [CtrlType] 
            表示末端位置，如左/右脚(Lfoot/Rfoot)，身体中心(Torso)
        - value: list, or two dim list. RPY, xyz.
            表示给相应末端的位姿值，[r, p, y, x, y, z]
        - count: int. times of division
            插值次数，缺省为 100
```



 ```python
   def reset(self):
       # 动作执行完后的复位，在动作完成后调用
 ```

要使用逆解接口实现机器人动作控制，需从ik_lib中导入 IkModuleSim 和 CtrlType，然后调用上述方法，设定机器人身体部位的末端位置，完成自定义动作，示例如下：

```Python

import math
import rospy
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C

class ClimbStairs(IkModuleSim):
    def __init__(self):
        super(ClimbStairs, self).__init__()

        def climb_stairs(self):
            # 重心下降 4cm
            self.body_motion(C.Torso, [0, 0, 0, 0, 0.0, -0.04])  
            # 重心向左腿偏移 12cm
            self.body_motion(C.Torso_y, 0.12)  
            # 右腿向上移动 5cm
            self.body_motion(C.Rfoot, [0, 0, 0, 0, 0.0, 0.05])  
            # 右腿向前20cm
            self.body_motion([C.Torso, C.Rfoot],
                             [[0, -math.pi / 8, 0, 0.0, 0, 0], 
                              [0, 0, 0, 0.20, 0.0, 0.0]]) 
            # 右腿下2cm
            self.body_motion([C.Torso, C.Rfoot],
                             [[0, math.pi / 8, 0, 0.08, -0.12, 0.0], 
                              [0, 0, 0, 0, 0.0, -0.02]])  
            # 抬左腿1
            self.body_motion([C.Torso, C.Lfoot], 
                             [[math.pi / 8, math.pi / 6, 0, 0.16, -0.10, 0.05],
                              [0, -math.pi / 6, 0, 0.06, 0.0, 0.14]])  
            # 左腿向前20cm
            self.body_motion([C.Torso, C.Lfoot], 
                             [[0.0, -math.pi / 9, 0, -0.04, 0.0, 0.0],
                              [0, math.pi / 6, 0, 0.14, 0.0, -0.05]])  
            # 复原
            self.body_motion([C.Torso, C.Lfoot],
                             [[-math.pi / 8, -math.pi / 18, 0, 0, 0.10, 0.02], 
                              [0, 0, 0, 0.0, 0.0, -0.06]]) 

```

  详细的使用示例可以参考`player_scripts/demo.py`示例程序。