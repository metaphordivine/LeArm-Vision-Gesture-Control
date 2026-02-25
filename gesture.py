import cv2
import mediapipe as mp
import time
import numpy as np
import socket # 改为 UDP 通信
import math
import struct

# 死区阈值（单位与你的控制量一致）
DEAD_ZONE_X = 0.5      # X轴变化小于0.5cm时不更新
DEAD_ZONE_Y = 0.5      # Y轴变化小于0.5cm时不更新
DEAD_ZONE_Z = 0.5      # Z轴变化小于0.5cm时不更新
DEAD_ZONE_C = 5        # 夹爪角度变化小于5度时不更新


# ================= 配置 IP =================
ESP32_IP = "192.168.43.10"  # 修改为你的 ESP32 IP
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= 逻辑函数 =================
def clamp(n, min_val, max_val): return max(min(max_val, n), min_val)

def is_index_bent(hand_landmarks, threshold_deg=148):
    """
    通过向量6->5与6->8的夹角判断食指是否弯曲
    :param hand_landmarks: MediaPipe 手部关键点对象
    :param threshold_deg: 弯曲角度阈值（度），夹角小于此值判定为弯曲
    :return: True 表示弯曲，False 表示伸直
    """
    # 获取关键点坐标（归一化坐标，0~1）
    x5 = hand_landmarks.landmark[5].x
    y5 = hand_landmarks.landmark[5].y
    x6 = hand_landmarks.landmark[6].x
    y6 = hand_landmarks.landmark[6].y
    x8 = hand_landmarks.landmark[8].x
    y8 = hand_landmarks.landmark[8].y

    # 向量6->5 和 6->8
    v1x = x5 - x6
    v1y = y5 - y6
    v2x = x8 - x6
    v2y = y8 - y6

    # 计算点积和模长
    dot = v1x * v2x + v1y * v2y
    norm1 = math.hypot(v1x, v1y)
    norm2 = math.hypot(v2x, v2y)

    if norm1 == 0 or norm2 == 0:
        return False  # 异常情况，视为伸直

    cos_angle = dot / (norm1 * norm2)
    # 防止浮点误差导致 acos 参数超出 [-1, 1]
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle_deg = math.degrees(math.acos(cos_angle))

    return angle_deg < threshold_deg

# 上次发送时间控制 (现在限制为 0.05秒，即 20 FPS)
last_send_time = 0
# 上次发送的值（初始值可设为None，或预设一个初始值）
_last_sent = {'x': None, 'y': None, 'z': None, 'c': None}

def send_udp(x, y, z, claw):
    global _last_sent, last_send_time
    cur_time = time.time()

    # 频率限制
    if cur_time - last_send_time < 0.05:  # 20 FPS
        return

    # 死区判断
    if _last_sent['x'] is not None:
        if (abs(x - _last_sent['x']) < DEAD_ZONE_X and
            abs(y - _last_sent['y']) < DEAD_ZONE_Y and
            abs(z - _last_sent['z']) < DEAD_ZONE_Z and
            abs(claw - _last_sent['c']) < DEAD_ZONE_C):
            last_send_time = cur_time
            return

    # 更新缓存
    _last_sent.update({'x': x, 'y': y, 'z': z, 'c': claw})

    # ⭐⭐⭐ 关键：构造二进制 UDP 包 ⭐⭐⭐
    packet = struct.pack(
        "<BB B B f f f f",
        0xAA, 0x55,     # 帧头
        1,              # mode=1（手势跟随）
        0,              # reserved
        float(x), float(y), float(z), float(claw)
    )

    sock.sendto(packet, (ESP32_IP, UDP_PORT))
    last_send_time = cur_time

def main():
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
    
    cap = cv2.VideoCapture(0)
    
    # 初始默认坐标 (机械臂的正前方)
    target_x, target_y, target_z = 15.0, 0.0, 10.0 
    claw_val = 90
    
    BEND_HISTORY_LEN = 5
    bend_history = []

    while True:
        success, frame = cap.read()
        if not success: break
        
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
            
            # 1. 提取手腕(0)的归一化坐标并映射为机械臂的 X, Y, Z
            wx, wy = hand.landmark[0].x, hand.landmark[0].y
        
            # Y轴左右(摄像头的X轴映射到机械臂的Y轴)
            # 画面最左边 wx=0 -> 机械臂 y=15; 画面最右边 wx=1 -> 机械臂 y=-15
            target_y = (0.5 - wx) * 35.0  
        
            # Z轴上下(摄像头的Y轴映射到机械臂的Z轴高度)
            # 画面最上边 wy=0 -> 机械臂 z=20; 画面最下边 wy=1 -> 机械臂 z=2
            target_z = max(-2, (0.95 - wy) * 20.0)
        
            # X轴前后(利用手掌大小作为距离Z映射到机械臂的X轴伸出长度)
            palm_size = abs(hand.landmark[0].y - hand.landmark[9].y)
            # 掌心大说明离得近(伸得长), 假设 palm_size 在 0.1(远) 到 0.3(近)
            target_x = 10.0 + (palm_size - 0.13) * 60.0 
            target_x = clamp(target_x, 5.0, 30.0)

            # 2. 食指弯曲检测控制夹爪 (加入滑动平均滤波)
            raw_bent = is_index_bent(hand)
            bend_history.append(raw_bent)
            if len(bend_history) > BEND_HISTORY_LEN:
                bend_history.pop(0)
            
            # 超过半数帧认为弯曲，才真正触发弯曲
            index_bent = (sum(bend_history) / len(bend_history)) > 0.5
            claw_val = 10 if index_bent else 90
            
            # 3. 通过 UDP 发送坐标
            send_udp(target_x, target_y, target_z, claw_val)
            
            cv2.putText(frame, f"Index bent: {index_bent}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.putText(frame, f"X:{target_x:.1f} Y:{target_y:.1f} Z:{target_z:.1f}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow('Fast IK Hand Control', frame)
        if cv2.waitKey(1) & 0xFF == 27: break
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()