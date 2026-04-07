🚗 RedStone_Smart_Car 智能小车项目
基于龙芯 LS2K0300 开发的视觉循迹智能小车
包含电机控制、编码器测速、陀螺仪稳向、视觉中线提取、串级 PD 循迹算法。
✨ 功能特性
视觉循迹（图像中线提取）
串级 PD 闭环控制
编码器速度 / 里程 / 加速度计算
DRV8701E 双路电机驱动
差速转向控制
10ms 系统定时中断
安全退出 + 资源自动清理
CMake 构建，跨平台编译
🧰 硬件平台
主控：龙芯 LS2K0300
电机驱动：DRV8701E
编码器：AB 相编码器
陀螺仪：MPU6050
摄像头：USB 摄像头 / 逐飞摄像头
📁 项目结构
plaintext
RedStone_raku_project/
├── code/            # 主程序代码
├── libraries/       # 驱动库
├── model/           # 模型/配置
├── resource/        # 资源文件
├── user/            # 用户代码
├── CMakeLists.txt   # 编译配置
└── README.md        # 项目说明
🚀 快速编译
bash
运行
mkdir build && cd build
cmake ..
make -j4
./project
🎯 主要模块
motor.cpp：电机驱动
encoder.cpp：编码器采集
control.cpp：串级 PD 循迹控制
image.cpp：视觉中线提取
pid.cpp：PID 算法
📌 开发者
rakurakudesu
GitHub：https://github.com/rakurakudesu
