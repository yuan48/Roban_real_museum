README for Linux SDK
-----------------------

编译依赖： cmake >= 3.0, gcc >= 4.9.0

注意：本 SDK 仅在 
	1. Intel x86_64 & i386 处理器
	2. ubutnu16.04 操作系统
	3. gcc5.4.0 编译链
下完整测试，编译成功后可放在其他 Linux 下运行

arm架构下 armv8a armv7a等等平台需要交叉编译，可在平台上提交工单！

为满足日益更新的跨平台编译，新增C接口以及V2版本接口，可不限于上述2,3
1. 将头文件 AIUI.h 更换为 AIUI_v2.h (不能都包含)
2. Buffer::alloc 操作换成 aiui_create_buffer_from_data 例如：

--------------------------------------------------------------

string txt = "合肥今天的天气怎么样";

Buffer *textData = Buffer::alloc(txt.length());
memcpy(textData->data(), txt.c_str(), txt.length());

<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

string txt = "合肥今天的天气怎么样";
AIUIBuffer textData = aiui_create_buffer_from_data(txt.c_str(), txt.length());

--------------------------------------------------------------
就可以通用了

目录结构说明

samples：
	 AIUI示例，展示如何使用aiui sdk进行文本语义，语音语义，以及动态上传资源


	libs：
	|-- x86
		|-- libaiui.so（32位动态库）
		|-- libvtn_mic1.so（单麦唤醒库）
	|-- x64
		|-- libaiui.so（64位动态库）
		|-- libvtn_mic1.so（单麦唤醒库）
		
	include:
		SDK 需要用到的头文件
		
	assets:
		sample程序运行所需要的配置
