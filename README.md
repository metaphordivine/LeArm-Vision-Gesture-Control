# LeArm-Vision-Gesture-Control
An advanced ESP32-based control system for LeArm (6DOF), featuring real-time hand gesture following via MediaPipe, HSV color-based sorting, and YOLOv8 intelligent object detection. Optimized with low-latency UDP communication and a multi-threaded video stream.
基于 ESP32 的 LeArm（6自由度）高级控制系统。集成了基于 MediaPipe 的实时手势跟随、基于 HSV 空间的角度颜色识别分拣、以及 YOLOv8 深度学习目标检测。采用低延迟 UDP 通信协议与多线程视频流优化。
🖐️ Gesture Control: Real-time 3D coordinate mapping using MediaPipe to follow hand movements with millisecond latency via UDP. (基于 MediaPipe 的实时手势三维坐标映射，通过 UDP 实现毫秒级随动)
🎨 Color Recognition: HSV-based color detection with a custom tuning UI and morphological filtering for stable sorting. (基于 HSV 空间的颜色识别，配备实时调参界面与形态学滤波，实现稳定分拣)
🔍 YOLO Intelligence: Integrated YOLOv8 model for advanced object classification and automated pick-and-place. (集成 YOLOv8 模型，实现高级物体分类与全自动抓取投放)
📐 Precise Calibration: Support for standard A4 paper calibration using homography matrix for cm-level accuracy. (支持标准 A4 纸单应性矩阵标定，实现厘米级抓取精度)
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

flowchart TD
    A1[摄像头视频流] --> A2[多线程读取 VideoStream]
    A2 --> A3[HSV 颜色检测<br>形态学处理]
    A2 --> A4[YOLOv8n 目标检测]
    A3 --> A5[目标筛选<br>类别/大小/稳定性]
    A4 --> A5
    A5 --> A6[像素坐标 (u,v)]
    A6 --> A7[透视变换矩阵 H]
    A7 --> A8[世界坐标 (Xw, Yw)]
    A8 --> A9[机械臂坐标系转换]

flowchart TD
    A1[PC 端发送 UDP/HTTP 指令<br>x,y,z,claw,mode] --> A2[ESP32 接收数据包]
    A2 --> A3[指令解析<br>模式判断]
    A3 --> A4[智能 IK 求解<br>自动 pitch 调整<br>手腕俯仰参与]
    A4 --> A5[舵机控制<br>PWM/串口]
    A5 --> A6[机械臂执行动作<br>抓取/放置]
