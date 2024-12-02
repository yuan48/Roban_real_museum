更新说明：

如果之前有在开放平台上下载我们的旧版本SDK，需要将该新版本集成到旧版本之中，除了将aiui.dll和aiui.lib替换之外，还要注意以下改动：

1、新版本头文件有改动，具体改动请自行对比下，主要是IAIUIListener类中的onEvent()方法的定义改动；

2、1069版本之后，aiui_sample支持vtn单麦及多麦唤醒
另外需在aiui.cfg中做如下两项配置：
	1)、speech->wakeup_mode的取值改为“vtn”, 默认情况下该值为“off”;
	2)、speech->audio_captor的取值改为“system”, 默认情况下该值为“off”;

3. 默认唤醒词为 “小飞小飞”， 若要更换唤醒词，需在平台资源制作中下载71版，将其中的 res.bin 覆盖掉 AIUI/assets/vtn/res.bin
