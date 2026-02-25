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

# 🏗️ 系统总体架构

```mermaid
flowchart LR
    subgraph PC[PC 端（Python）]
        A1[摄像头视频流<br>ESP32-CAM] --> A2[多线程读取 VideoStream]
        A2 --> A3[视觉检测<br>YOLOv8 / HSV]
        A3 --> A4[像素坐标 (u,v)]
        A4 --> A5[透视变换矩阵 H<br>标定]
        A5 --> A6[世界坐标 (Xw, Yw)]
        A6 --> A7[机械臂坐标系映射 (X,Y,Z)]
        A7 --> A8[UDP/HTTP 指令发送]
    end

    A8 -- WiFi --> B1

    subgraph ESP32[ESP32 机械臂控制端]
        B1[UDP/HTTP 指令解析] --> B2[智能 IK 姿态求解<br>6DOF]
        B2 --> B3[舵机控制<br>PWM/串口]
        B3 --> B4[机械臂执行动作]
    end
