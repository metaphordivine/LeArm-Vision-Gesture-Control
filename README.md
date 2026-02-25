# LeArm-Vision-Gesture-Control
An advanced ESP32-based control system for LeArm (6DOF), featuring real-time hand gesture following via MediaPipe, HSV color-based sorting, and YOLOv8 intelligent object detection. Optimized with low-latency UDP communication and a multi-threaded video stream.
基于 ESP32 的 LeArm（6自由度）高级控制系统。集成了基于 MediaPipe 的实时手势跟随、基于 HSV 空间的角度颜色识别分拣、以及 YOLOv8 深度学习目标检测。采用低延迟 UDP 通信协议与多线程视频流优化。
🖐️ Gesture Control: Real-time 3D coordinate mapping using MediaPipe to follow hand movements with millisecond latency via UDP. (基于 MediaPipe 的实时手势三维坐标映射，通过 UDP 实现毫秒级随动)
🎨 Color Recognition: HSV-based color detection with a custom tuning UI and morphological filtering for stable sorting. (基于 HSV 空间的颜色识别，配备实时调参界面与形态学滤波，实现稳定分拣)
🔍 YOLO Intelligence: Integrated YOLOv8 model for advanced object classification and automated pick-and-place. (集成 YOLOv8 模型，实现高级物体分类与全自动抓取投放)
📐 Precise Calibration: Support for standard A4 paper calibration using homography matrix for cm-level accuracy. (支持标准 A4 纸单应性矩阵标定，实现厘米级抓取精度)
