# 纯惯导动态导航定位实验

**姓名：** GYH   
**课程：** 惯性导航原理  
**实验名称：** 实验三 - 纯惯导动态导航定位实验

---

## 📋 项目概述

本项目实现了基于IMU（惯性测量单元）的纯惯导动态导航定位系统。系统通过读取IMU原始数据（加速度计和陀螺仪数据），采用四元数方法进行姿态更新，并实现速度更新和位置更新，最终输出导航定位结果。项目支持两种运行模式：示例数据模式和自测数据模式。

### 主要功能

- ✅ IMU原始数据读取与解析
- ✅ 加速度计和陀螺仪标定（零偏补偿）
- ✅ 基于四元数的姿态更新算法
- ✅ 速度更新算法（考虑地球自转和载体运动）
- ✅ 位置更新算法（BLH坐标系）
- ✅ 坐标转换（BLH ↔ XYZ ↔ ENU）
- ✅ 导航结果与真值对比分析
- ✅ 轨迹误差计算与可视化

---

## 📁 项目结构

```
纯惯导动态导航定位实验/
│
├── 代码/
│   └── Pure Inertial Dynamic Navigation/
│       ├── Pure Inertial Dynamic Navigation.sln          # Visual Studio解决方案文件
│       └── Pure Inertial Dynamic Navigation/
│           ├── main.cpp                                  # 主程序入口
│           ├── IMU_Structs.h                             # 数据结构定义和函数声明
│           ├── ReadFile.cpp                              # 数据读取模块
│           ├── Calibration.cpp                           # 标定模块（零偏补偿）
│           ├── NavigationUpdate.cpp                      # 导航更新模块（姿态/速度/位置）
│           ├── Calculations.cpp                          # 四元数计算、重力计算等
│           ├── CoordinateChange.cpp                      # 坐标转换模块
│           ├── SaveFile.cpp                              # 结果保存模块
│           ├── MatrixPrint.cpp                           # 矩阵打印工具
│           ├── Eigen-3.4.props                           # Eigen库配置
│           │
│           ├── Data/                                     # 数据文件夹
│           │   ├── Example_Data/                         # 示例数据
│           │   │   ├── IMU.bin                           # 示例IMU原始数据（二进制）
│           │   │   ├── IMU.txt                           # 示例IMU原始数据（文本）
│           │   │   ├── PureINS.bin                       # 示例INS参考数据（二进制）
│           │   │   └── INS.txt                           # 示例INS参考数据（文本）
│           │   ├── GroupOne.ASC                          # 自测IMU数据文件
│           │   ├── TruthOne.nav                          # 自测真值数据文件
│           │   ├── truth.txt                             # 真值数据（文本格式）
│           │   └── truth格式.png                         # 真值数据格式说明
│           │
│           └── Result/                                   # 结果文件夹
│               ├── result.txt                            # 自测数据导航结果
│               ├── true.txt                              # 自测数据真值
│               ├── diff_result.txt                       # 自测数据误差结果
│               ├── denu_result.txt                       # 自测数据ENU轨迹误差
│               ├── ExampleResult/                        # 示例数据结果
│               │   ├── INS_Result.txt                    # 示例数据导航结果
│               │   ├── INS_True.txt                      # 示例数据真值
│               │   ├── INS_Diff_Result.txt               # 示例数据误差结果
│               │   └── denu_Result.txt                   # 示例数据ENU轨迹误差
│               ├── Figures/                              # 结果图表
│               │   ├── Example/                          # 示例数据图表
│               │   ├── RawResults/                       # 原始结果图表
│               │   ├── ZeroResults/                      # 零速修正结果图表
│               │   └── CaliResults/                      # 标定结果图表
│               └── *.m                                   # MATLAB绘图脚本
│
└── 实验报告/
    ├── 实验报告.docx                         # 实验报告（Word格式）
    ├── 实验报告.pdf                          # 实验报告（PDF格式）
    └── 图例.pptx                           # 报告图例
```

---

## 🔧 环境配置

### 开发环境要求

- **IDE：** Visual Studio 2019/2022
- **编程语言：** C++17
- **依赖库：** Eigen 3.4（矩阵运算库）

### 依赖安装

1. **安装Visual Studio**
   - 下载并安装Visual Studio 2019或更高版本
   - 安装时选择"使用C++的桌面开发"工作负载

2. **配置Eigen库**
   - 项目已包含`Eigen-3.4.props`配置文件，通常无需手动配置
   - 如遇编译错误，请检查Eigen库路径是否正确

---

## 🚀 使用说明

### 快速开始

1. **打开项目**
   - 双击`代码/Pure Inertial Dynamic Navigation/Pure Inertial Dynamic Navigation.sln`打开解决方案
   - 在Visual Studio中选择`x64`平台和`Debug`或`Release`配置

2. **编译项目**
   - 点击菜单栏"生成" → "生成解决方案"，或按快捷键`F7`
   - 等待编译完成，确保无错误

3. **运行程序**
   - 点击菜单栏"调试" → "开始调试"，或按快捷键`F5`
   - 或者直接运行`x64/Debug/Pure Inertial Dynamic Navigation.exe`
   - 程序启动后会提示选择运行模式：
     ```
     Please choose the mode of solution:
       0 ExampleData
       1 SelfData
     ```
   - 输入`0`运行示例数据模式
   - 输入`1`运行自测数据模式

### 运行模式说明

#### 模式0：示例数据模式（ExampleData）
- 读取`Data/Example_Data/`目录下的示例数据
- 输出结果保存到`Result/ExampleResult/`目录
- 适用于验证算法正确性

#### 模式1：自测数据模式（SelfData）
- 读取`Data/GroupOne.ASC`（IMU数据）和`Data/TruthOne.nav`（真值数据）
- 输出结果保存到`Result/`目录
- 支持零速修正功能（可选）
- 支持加速度计标定功能（可选）

### 输出结果说明

程序会生成以下结果文件：

1. **result.txt / INS_Result.txt**
   - 格式：`时间戳 纬度(deg) 经度(deg) 高程(m) 北向速度(m/s) 东向速度(m/s) 天向速度(m/s) 横滚角(deg) 俯仰角(deg) 航向角(deg)`

2. **true.txt / INS_True.txt**
   - 真值数据，格式同上

3. **diff_result.txt / INS_Diff_Result.txt**
   - 误差分析结果
   - 格式：`时间戳 速度误差(m/s) 位置误差(deg/m) 姿态误差(deg)`

4. **denu_result.txt / denu_Result.txt**
   - ENU坐标系下的轨迹误差
   - 格式：`时间戳 真值dN(m) 真值dE(m) 真值dU(m) 计算值dN(m) 计算值dE(m) 计算值dU(m)`

---

## 📊 结果可视化

### MATLAB绘图

项目提供了MATLAB脚本用于结果可视化，位于`Result/`目录下：

- **main.m**：主绘图脚本，生成轨迹、误差等图表
- **DiffFig.m**：误差对比图
- **dEnuFig.m**：ENU误差图
- **RmseWithTime.m**：RMSE随时间变化图
- **DataFig.m**：数据可视化

### 运行MATLAB脚本

1. 打开MATLAB
2. 切换到`Result/`目录
3. 运行`main.m`或其他脚本
4. 生成的图表保存在`Result/Figures/`目录下（SVG格式）

---

## 🔬 算法原理

### 1. 姿态更新（四元数方法）

采用四元数方法进行姿态更新，避免欧拉角的万向锁问题：

- 计算载体坐标系（b系）的有效旋转矢量
- 计算导航坐标系（n系）的有效旋转矢量（考虑地球自转和载体运动）
- 通过四元数乘法更新姿态四元数
- 从四元数提取欧拉角（横滚、俯仰、航向）

### 2. 速度更新

考虑以下因素：
- 比力积分
- 重力加速度
- 地球自转引起的科里奥利加速度
- 载体运动引起的向心加速度

### 3. 位置更新

在BLH（大地坐标系）下进行位置更新：
- 根据速度计算纬度和经度的变化率
- 考虑地球曲率的影响
- 更新高程

### 4. 坐标转换

实现以下坐标转换：
- **BLH ↔ XYZ**：大地坐标与地心直角坐标转换
- **XYZ ↔ ENU**：地心坐标与站心坐标转换

### 5. 标定算法

- **加速度计标定**：计算零偏并补偿
- **陀螺仪标定**：计算零偏并补偿（可选）

---

## 📝 关键参数配置

主要参数在`IMU_Structs.h`中定义：

### WGS84椭球参数
```cpp
#define a 6378137.0              // 长半轴(m)
#define b 6356752.3142           // 短半轴(m)
#define we 7.292115e-5           // 地球自转角速度(rad/s)
#define gravity 9.7936174        // 重力加速度
```

### 示例数据初始条件
```cpp
const double initial_latitude = 23.1373950708 * Rad;
const double initial_longitude = 113.3713651222 * Rad;
const double initial_elevation = 2.175;
```

### 自测数据初始条件
```cpp
const double ours_initial_latitude = 30.5279685193 * Rad;
const double ours_initial_longitude = 114.3555367313 * Rad;
const double ours_initial_height = 23.3659979239;
```

### 设备参数
```cpp
#define rate_CGI 100                    // CGI设备采样频率(Hz)
#define acc_CGI 1.0/655360.0            // 加速度计标度因数
#define gyr_CGI 1.0/160849.543863       // 陀螺仪标度因数
```

---

## 📈 实验结果

实验结果保存在`Result/`目录下，包括：

1. **导航结果**：位置、速度、姿态的时间序列
2. **误差分析**：与真值对比的误差统计
3. **轨迹对比**：ENU坐标系下的轨迹误差
4. **可视化图表**：SVG格式的各类分析图表

详细结果分析请参考实验报告。

---

## 📚 代码模块说明

### main.cpp
主程序入口，包含：
- 模式选择（示例数据/自测数据）
- 数据读取循环
- 导航解算流程控制
- 结果保存

### IMU_Structs.h
核心头文件，包含：
- 数据结构定义（IMU数据、INS数据、四元数等）
- 常量定义（WGS84参数、初始条件等）
- 函数声明

### ReadFile.cpp
数据读取模块：
- `ReadExamplePureIMUData()`：读取示例IMU数据
- `ReadIMURawData_CGI()`：读取自测IMU数据
- `ReadTruthData()`：读取真值数据

### Calibration.cpp
标定模块：
- `CalAvgAcc_Gyr()`：计算加速度计和陀螺仪平均值
- `AccCalibration()`：加速度计零偏标定
- `GyrCalibration()`：陀螺仪零偏标定

### NavigationUpdate.cpp
导航更新模块：
- `PostureUpdate()`：姿态更新
- `VelocityUpdate()`：速度更新
- `PositionUpdate()`：位置更新

### Calculations.cpp
计算工具模块：
- `QuaternionMultiply()`：四元数乘法
- `CalPostureWithQuaternion()`：从四元数提取姿态
- `Calgpn()`：计算重力加速度
- `Extrapolation()`：外推计算

### CoordinateChange.cpp
坐标转换模块：
- `BLHToXYZ()`：BLH转XYZ
- `XYZToBLH()`：XYZ转BLH
- `CompEnudPos()`：计算ENU坐标

### SaveFile.cpp
结果保存模块：
- `SaveOurResult()`：保存导航结果
- `SaveTrueResult()`：保存真值
- `SaveDiffResult()`：保存误差结果
- `SavedENUResult()`：保存ENU轨迹误差

---

## ⚠️ 注意事项

1. **数据路径**：确保数据文件路径正确，程序运行时需要在正确的目录下执行
2. **时间同步**：自测数据模式下，确保IMU数据与真值数据时间戳同步
3. **初始条件**：根据实际数据修改`IMU_Structs.h`中的初始条件参数
4. **零速修正**：自测数据模式支持零速修正，需要在代码中设置零速时间段
5. **标定功能**：标定功能需要在数据采集前进行静态标定

---

## 📄 实验报告

完整的实验报告位于`实验报告/`目录下：
- **实验报告.docx**：Word格式报告
- **实验报告.pdf**：PDF格式报告
- **图例.pptx**：报告中的图例说明

报告内容包括：
- 实验目的与原理
- 算法实现细节
- 实验结果分析
- 误差分析与讨论
- 结论与总结

---

## 👤 作者信息

- **姓名：** GYH
- **课程：** 惯性导航原理
- **实验：** 实验三 - 纯惯导动态导航定位实验

---

## 📌 版本说明

- **版本：** 1.0
- **最后更新：** 2024年

---

## 🙏 致谢

感谢课程老师提供的实验指导和示例数据，以及Eigen库开发团队提供的优秀矩阵运算库。

---

**注意：** 本项目为课程作业，仅供学习和参考使用。

