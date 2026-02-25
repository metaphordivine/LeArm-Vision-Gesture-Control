# 🌟 LeArm Vision & Gesture Control System  
基于 YOLOv8 + MediaPipe + ESP32 的 6DOF 机械臂智能视觉抓取与手势控制系统

---

## 🧩 技术栈

![Python](https://img.shields.io/badge/Python-3.10-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8-green)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Detection-red)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-yellow)
![ESP32](https://img.shields.io/badge/ESP32-WiFi-orange)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

# 🚀 项目简介

本项目实现了：

- 🎯 **YOLOv8 俯视视觉抓取（支持小物体 < 5cm）**
- 🖐️ **MediaPipe 手势控制机械臂（实时跟随）**
- 🎨 **HSV 颜色分拣（红绿蓝）**
- 📡 **UDP + HTTP 双通道通信**
- 🤖 **智能 IK 姿态控制（Z→pitch 自动调整）**
- 📐 **A4 纸四点透视标定（像素→世界坐标）**

这是一个完整的 **视觉 + 控制 + 机械臂** 工程项目。

---

🏗️ 系统总体架构
```mermaid
flowchart TD
    subgraph PC["PC 端（Python）"]
        A1["摄像头视频流<br>ESP32-CAM"] --> A2["多线程读取 VideoStream"]
        A2 --> A3["视觉检测<br>YOLOv8 / HSV"]
        A3 --> A4["像素坐标 (u,v)"]
        A4 --> A5["透视变换矩阵 H<br>标定"]
        A5 --> A6["世界坐标 (Xw, Yw)"]
        A6 --> A7["机械臂坐标系映射 (X,Y,Z)"]
        A7 --> A8["UDP/HTTP 指令发送"]
    end

    A8 ==>|WiFi| B1

    subgraph ESP32["ESP32 机械臂控制端"]
        B1["UDP/HTTP 指令解析"] --> B2["智能 IK 姿态求解<br>6DOF"]
        B2 --> B3["舵机控制<br>PWM/串口"]
        B3 --> B4["机械臂执行动作"]
    end

```
---
👁️ 视觉处理流程
```mermaid
flowchart TD
    A1["摄像头视频流"] --> A2["多线程读取 VideoStream"]
    A2 --> A3["HSV 颜色检测<br>形态学处理"]
    A2 --> A4["YOLOv8n 目标检测"]
    A3 --> A5["目标筛选<br>类别/大小/稳定性"]
    A4 --> A5
    A5 --> A6["像素坐标 (u,v)"]
    A6 --> A7["透视变换矩阵 H"]
    A7 --> A8["世界坐标 (Xw, Yw)"]
    A8 --> A9["机械臂坐标系转换"]

```
---
🤖 机械臂控制流程
```mermaid
flowchart TD
    A1["PC 端发送 UDP/HTTP 指令<br>x,y,z,claw,mode"] --> A2["ESP32 接收数据包"]
    A2 --> A3["指令解析<br>模式判断"]
    A3 --> A4["智能 IK 求解<br>自动 pitch 调整<br>手腕俯仰参与"]
    A4 --> A5["舵机控制<br>PWM/串口"]
    A5 --> A6["机械臂执行动作<br>抓取/放置"]
```
✨ 功能亮点
🔹 YOLOv8 俯视抓取

🔹 MediaPipe 手势控制
手腕位置 → X/Y/Z

食指弯曲 → 夹爪开合

死区 + 滤波 → 稳定控制

🔹 智能 IK 姿态控制（6DOF）
Z 低 → 垂直向下抓取

Z 高 → 更水平，避免无解

手腕俯仰自由参与 IK

🔹 透视标定（A4 纸）
点击四角 → 自动求单应矩阵

像素坐标 → 世界坐标（cm）

机械臂坐标系自动对齐

