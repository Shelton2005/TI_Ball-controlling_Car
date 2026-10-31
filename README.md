# 电赛控球小车｜代码与实物演示

基于 **TI MSPM0G3507** 的嵌入式控球小车工程，包含八路灰度循迹、编码器速度反馈、IMU 航向处理、分题任务状态机与分段舵机控制。

**技术栈：C · MSPM0G3507 · Keil MDK · ARM Compiler 6 · IMU660RA · TB6612**

> An embedded ball-controlling car project on TI MSPM0G3507, featuring eight-channel line sensing, encoder feedback, IMU heading processing, task state machines, and a timed servo control sequence.

## 实物演示

[查看或下载演示视频](成果演示/015624046d14a752e0f60c5d4683fe33.mp4)（约 2.1 MB）

视频保留原文件。具体任务名称、拍摄条件与效果指标尚待补充；当前不将该视频视为全部任务的验证结果。

## 核心实现与代码入口

| 模块 | 内容 | 代码入口 |
| --- | --- | --- |
| 启动与任务调度 | 硬件初始化、1 ms 系统节拍、按键与菜单、5 ms / 10 ms 任务调用 | [main.c](project/user/src/main.c) |
| 灰度传感器 | 八路模拟灰度采集、CD4051 通道选择、黑白标定及数字状态生成 | [Gray_sensor.c](project/code/Gray_sensor.c)、[Gray_sensor.h](project/code/Gray_sensor.h) |
| 循迹控制 | 路段状态、转向修正、左右轮目标及速度控制、编码器异常状态 | [Questoin_2.c](project/code/Questoin_2.c)、[Questoin_2.h](project/code/Questoin_2.h) |
| 分段舵机控制 | 根据绝对运行时间插值生成舵机目标，按阶段推进控制时序 | [Question_3.c](project/code/Question_3.c) |
| 第四题任务 | 加速、巡航、减速与停车阶段，距离和航向状态，以及异常处理 | [Question_4.c](project/code/Question_4.c) |
| 第五题任务 | 任务启停、循迹参数配置及周期控制接口 | [Question5.c](project/code/Question5.c) |
| 编码器反馈 | 编码器采集与反馈处理 | [Encoder.c](project/code/Encoder.c) |
| IMU | 姿态与航向相关处理 | [Imu.c](project/code/Imu.c) |
| 外设与交互 | 电机、舵机、按键、屏幕菜单 | [Common_peripherals.c](project/code/Common_peripherals.c)、[Menu.c](project/code/Menu.c) |
| 算法参数 | PID 参数结构与初始化 | [Agorithm.c](project/code/Agorithm.c) |

建议阅读顺序：`main.c` → 对应题目控制文件 → `Gray_sensor.c` / `Encoder.c` / `Imu.c` → 外设与底层库。

为保持 Keil 工程引用有效，保留源码中的 `Questoin_2` 和 `Agorithm` 等现有文件名。

## 控制设计

- **循迹与速度反馈：** 通过灰度状态形成转向修正，再结合左右轮目标与编码器反馈控制车辆运动。代码包含直线、弯道、间断区域、搜索等状态与独立参数。
- **任务状态机：** 各题使用独立的启动、停止与周期执行接口，主循环统一调度，并将完成和异常状态交给菜单显示。
- **分段舵机时序：** 第三题采用按时间推进的开环控制轨迹，通过插值生成各阶段目标。当前代码不包含对球位置的视觉闭环反馈。
- **调试入口：** 菜单、灰度标定、编码器与航向状态用于实车调试；各题的控制结构和启用功能应分别核对。

上述内容描述当前代码，尚未整理误差曲线、成功率或调参前后对比，不作为量化性能结论。

## 工程结构

```text
.
├── README.md
├── libraries/                  # TI SDK、逐飞驱动及组件
├── project/
│   ├── code/                   # 业务算法、传感器和各题任务
│   ├── user/src/               # main.c 与中断入口
│   ├── user/inc/               # 用户头文件
│   └── keil/                  # Keil 工程与链接配置
└── 成果演示/                    # 原始演示视频
```

## 开发与复现

1. 用 Keil MDK 打开 [SeekFree_MSPM0G3507_Device_Library.uvprojx](project/keil/SeekFree_MSPM0G3507_Device_Library.uvprojx)。现有工程配置记录的编译器为 **ARMCLANG V6.19**，设备包为 **TexasInstruments.MSPM0G1X0X_G3X0X_DFP.1.3.1**，目标芯片为 **MSPM0G3507**。
2. 保持 `project/` 与 `libraries/` 的相对位置。核对设备包、编译器及调试器配置后构建工程。
3. 检查 `Gray_sensor.h`、`Common_peripherals.h`、`Encoder.h` 与 `Imu.h` 的接线、反馈方向及硬件参数。当前主程序初始化 IMU660RA、TB6612 和 SPI IPS200。
4. 当前 `main.c` 中 `SERVO_TEST_MODE=0`、`GRAY_SENSOR_TEST_MODE=1`，启动路径会调用 `Gray_sensor_test()`。请先核对灰度测试与标定流程，再验证菜单中的正式任务。
5. 下载至配套硬件，完成灰度黑白标定、IMU 零偏和控制参数校准，按任务分别测试。

本次仓库整理仅验证文档链接及工程文件引用，未执行 Keil 编译、烧录或实车复测。当前上传内容是现有源码快照，不保证与演示视频拍摄时的固件版本完全一致。

## 来源与贡献说明

`libraries/` 包含 TI SDK、逐飞等第三方代码，保留其原有版权和许可声明；第三方底层代码不属于个人独立开发内容。

比赛正式名称、年份、成绩、本人具体职责以及视频对应题目尚待补充。
