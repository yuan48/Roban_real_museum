#!/usr/bin python3
# coding=utf-8
from enum import IntEnum

class AIUIConstant(IntEnum):
    """aiui事件和消息相关常量"""

    EVENT_RESULT = 1
    """结果事件
    data 字段携带结果数据，info 字段为描述数据的 JSON 字符串
    """

    EVENT_ERROR = 2
    """出错事件
    arg1 字段为错误码，info字段为错误描述信息
    """

    EVENT_STATE = 3
    """服务状态事件
    arg1 为服务状态，取值：STATE_IDLE、STATE_READY、STATE_WORKING
    """

    EVENT_WAKEUP = 4
    """唤醒事件
    arg1 字段取值：
        0 => 内部语音唤醒
        1 => 外部手动唤醒
    info 字段为唤醒结果 JSON 字符串
    """

    EVENT_SLEEP = 5
    """休眠事件
    出现交互超，服务会进入休眠状态（待唤醒），或者发送了 CMD_RESET_WAKEUP 时，抛出该事件
    arg1 字段取值：
        0 => TYPE_AUTO（自动休眠，即交互超时）
        1 => TYPE_COMPEL（外部强制休眠，即发送 CMD_RESET_WAKEUP）
    """

    EVENT_VAD = 6
    """ VAD 事件
    当检测到输入音频的前端点后，会抛出该事件，用 arg1 标识前后端点或音量信息：
        0 => 前端点
        1 => 音量
        2 => 后端点
        3 => 前端点超时
    当 arg1 取值为 1 时，arg2 为音量大小，取值范围：[0-30]
    """

    EVENT_CMD_RETURN = 8
    """某条 CMD 命令对应的返回事件
    对于除 CMD_GET_STATE 外的有返回的命令，会返回该事件
    用 arg1 标识对应的CMD命令，arg2 为返回值，0 表示成功，info 字段为描述信息
    """

    EVENT_PRE_SLEEP = 10
    """准备休眠事件
    当出现交互超时，服务会先抛出准备休眠事件，用户可在收到该事件后 10s 内继续交互，10s 后进入休眠状态
    """

    EVENT_START_RECORD = 11
    """抛出该事件通知外部录音开始，用户可以开始说话"""

    EVENT_STOP_RECORD = 12
    """通知外部录音停止"""

    EVENT_CONNECTED_TO_SERVER = 13
    """与服务端建立起连接事件
    连接建立后，才能进行数据同步等操作
    """

    EVENT_SERVER_DISCONNECTED = 14
    """与服务端断开连接事件
    连接断开后，将不能进行数据同步等操作
    """

    EVENT_TTS = 15
    """语音合成事件"""

    STATE_IDLE = 1
    """空闲状态，AIUI服务未开启"""

    STATE_READY = 2
    """就绪状态，等待唤醒"""

    STATE_WORKING = 3
    """工作状态，已经唤醒，可以开始人机交互"""

    CMD_GET_STATE = 1
    """获取交互状态
    AIUI 会回应 EVENT_STATE 事件
    """

    CMD_WRITE = 2
    """写入数据"""

    CMD_STOP_WRITE = 3
    """停止写入数据"""

    CMD_RESET = 4
    """重置 AIUI 服务的状态
    服务会立即停止并重新启动，进入到待唤醒状态
    """

    CMD_START = 5
    """启动 AIUI 服务"""

    CMD_STOP = 6
    """停止 AIUI 服务"""

    CMD_WAKEUP = 7
    """手动唤醒"""

    CMD_RESET_WAKEUP = 8
    """休眠消息
    服务重置为待唤醒状态，若当前为唤醒状态，发送该消息重置后会抛出 EVENT_SLEEP 事件
    """

    CMD_SET_PARAMS = 10
    """设置参数配置
    用 params 携带参数设置 JSON 字符串，具体格式参照 aiui.cfg 文件
    """

    CMD_SYNC = 13
    """同步个性化数据
    arg1 表示同步的数据类型
    data 表示同步的数据内容
    """

    CMD_RESULT_VALIDATION_ACK = 20
    """
    结果确认
    在接收到语义、听写、后处理的结果后 5s 内发送该指令对结果进行确认，AIUI会认为该条结果有效，并重新开始 AIUI 交互超时的计时
    """

    CMD_CLEAN_DIALOG_HISTORY = 21
    """清除云端语义对话历史"""

    CMD_QUERY_SYNC_STATUS = 24
    """查询数据同步状态
    arg1 表示状态查询的类型
    params 包含查询条件，需要在 params 中通过 sid 字段指定 CMD_SYNC 返回的 sid
    """

    CMD_TTS = 27
    """进行语音合成
    arg1 表示控制语音合成命令
    params 包含合成参数
    """

    SUCCESS = 0
    """成功"""

    FAIL = -1
    """失败"""

    VAD_BOS = 0
    """VAD 消息类型，前端点"""

    VAD_VOL = 1
    """VAD 消息类型，音量"""

    VAD_EOS = 2
    """VAD 消息类型，后端点"""

    VAD_BOS_TIMEOUT = 3
    """VAD 消息类型，前端点超时"""

    TYPE_AUTO = 0
    """自动休眠"""

    TYPE_COMPEL = 1
    """外部强制休眠"""

