import cv2
import numpy as np
import math
import requests
import time
import threading

# ================= 配置 IP =================
EYE_IP = "192.168.43.8"    # ESP32-CAM IP
ARM_IP = "192.168.43.10"    # ESP32 机械臂 IP
STREAM_URL = f"http://{EYE_IP}:81/stream"

http_session = requests.Session()

# ================= 机械臂环境参数 =================
TABLE_Z = 5.0    # 抓取高度
HOVER_Z = 19.0   # 悬停安全高度

# ⭐ 标定矩形真实尺寸 (A4纸竖放)
CALIB_WIDTH = 21.0  # X轴方向长度
CALIB_HEIGHT = 29.7 # Y轴方向长度

# ⭐ 分拣投放目标坐标 (X, Y)
DROP_POSITIONS = {
    'orange':    (10.0, 15.0),   # 左边
    'Green':  (10.0, -15.0),  # 右边
    'Blue':   (20.0, 0.0)     # 正前方
}

# ================= 颜色阈值 (初始值) =================
COLOR_RANGES = {
    'orange':   ([0, 43, 46],[10, 255, 255]),
    'Green': ([35, 43, 46],[78, 255, 255]),
    'Blue':  ([80, 43, 46],[140, 255, 255])
}

# 全局变量
calib_points =[]
base_point_pixel = None 
base_pos_world = None   
matrix_pixel_to_world = None 
state = 0 
detected_objects =[] 
current_frame = None # 用于鼠标右键取色
debug_text = ""      # 调试信息

# ================= 多线程无阻塞视频流 =================
class VideoStream:
    def __init__(self, src=0):
        self.src = src
        self.stream = cv2.VideoCapture(src)
        if self.stream.isOpened():
            self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.stopped = False
        self.frame = None
        self.grabbed = False
        self.lock = threading.Lock()

    def start(self):
        t = threading.Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while not self.stopped:
            if not self.stream.isOpened():
                time.sleep(1)
                try: self.stream.open(self.src)
                except: pass
                continue
            
            (grabbed, frame) = self.stream.read()
            with self.lock:
                if grabbed:
                    self.grabbed = grabbed
                    self.frame = frame
                else:
                    self.stream.release()

    def read(self):
        with self.lock: return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

# ================= 核心控制逻辑 =================
def send_coords_to_esp32(x, y, z, claw_val, move_time=1000, max_retries=3, retry_delay=0.1):
    """发送坐标指令给 ESP32，带重试机制"""
    params = {
        'x': round(float(x), 2),
        'y': round(float(y), 2),
        'z': round(float(z), 2),
        'c': int(claw_val),
        't': int(move_time)
    }
    url = f"http://{ARM_IP}/robot"
    
    for attempt in range(1, max_retries + 2):
        try:
            http_session.get(url, params=params, timeout=0.3)
            print(f"指令发送成功: {params}")
            return True
        except requests.exceptions.RequestException:
            if attempt <= max_retries: time.sleep(retry_delay)
            else:
                print(">>> 发送失败，放弃该指令")
                return False

def execute_grab_sequence(x, y, color_name):
    CLAW_OPEN, CLAW_CLOSE = 90, 10
    drop_x, drop_y = DROP_POSITIONS.get(color_name, (15.0, 0.0))
    
    print(f"\n--- 开始抓取[{color_name}] ---")
    
    def step(action_name, *args, **kwargs):
        print(f"{action_name}...")
        if not send_coords_to_esp32(*args, **kwargs):
            print(f"!!! 步骤 '{action_name}' 失败，终止序列")
            return False
        
        # ⭐ 修复防假死：用 cv2.waitKey 替代 time.sleep，保持 GUI 不崩溃
        t = args[4] if len(args) >= 5 else kwargs.get('move_time', 1000)
        delay_ms = int(t + 200)
        cv2.waitKey(delay_ms) 
        return True
    
    if not step("1. 悬停对准", x, y, HOVER_Z, CLAW_OPEN, 1000): return
    if not step("2. 下降", x, y, TABLE_Z, CLAW_OPEN, 800): return
    if not step("3. 抓取", x, y, TABLE_Z, CLAW_CLOSE, 500): return
    if not step("4. 提起", x, y, HOVER_Z, CLAW_CLOSE, 800): return
    if not step("5. 移动投放", drop_x, drop_y, HOVER_Z, CLAW_CLOSE, 1500): return
    if not step("6. 松开投放", drop_x, drop_y, HOVER_Z, CLAW_OPEN, 500): return
    if not step("7. 归位待命", 5.0, 0.0, HOVER_Z+5, CLAW_OPEN, 1200): return
    
    print("--- 序列完成 ---\n")

def process_target(u, v, color_name):
    if matrix_pixel_to_world is None: return
    vec = np.array([[[u, v]]], dtype=np.float32)
    target_pos = cv2.perspectiveTransform(vec, matrix_pixel_to_world)[0][0]

    rob_x = target_pos[0] - base_pos_world[0]
    rob_y = target_pos[1] - base_pos_world[1]
    
    if rob_x < 5.0:
        print(">>> 目标太近或在后方，拒绝抓取！")
        return
        
    execute_grab_sequence(rob_x, rob_y, color_name)

# ================= 视觉处理与UI =================
def setup_trackbars():
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 300, 600)
    cv2.createTrackbar("Brightness", "Settings", 25, 100, lambda x: None)
    
    cv2.createTrackbar("orange H Min", "Settings", 0, 179, lambda x: None)
    cv2.createTrackbar("orange H Max", "Settings", 20, 179, lambda x: None)
    cv2.createTrackbar("Grn H Min", "Settings", 35, 179, lambda x: None)
    cv2.createTrackbar("Grn H Max", "Settings", 78, 179, lambda x: None)
    cv2.createTrackbar("Blu H Min", "Settings", 80, 179, lambda x: None)
    cv2.createTrackbar("Blu H Max", "Settings", 119, 179, lambda x: None)
    
    cv2.createTrackbar("Min S", "Settings", 45, 255, lambda x: None)
    cv2.createTrackbar("Min V", "Settings", 80, 255, lambda x: None)

def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (7, 7), 0)
    
    objects = []
    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    
    try:
        min_s = cv2.getTrackbarPos("Min S", "Settings")
        min_v = cv2.getTrackbarPos("Min V", "Settings")
        r_min, r_max = cv2.getTrackbarPos("orange H Min", "Settings"), cv2.getTrackbarPos("orange H Max", "Settings")
        g_min, g_max = cv2.getTrackbarPos("Grn H Min", "Settings"), cv2.getTrackbarPos("Grn H Max", "Settings")
        b_min, b_max = cv2.getTrackbarPos("Blu H Min", "Settings"), cv2.getTrackbarPos("Blu H Max", "Settings")
    except: return []

    COLOR_RANGES['orange']   = ([r_min, min_s, min_v],[r_max, 255, 255])
    COLOR_RANGES['Green'] = ([g_min, min_s, min_v], [g_max, 255, 255])
    COLOR_RANGES['Blue']  = ([b_min, min_s, min_v],[b_max, 255, 255])
    
    for name, (low, high) in COLOR_RANGES.items():
        if name == 'orange':
            mask1 = cv2.inRange(hsv, np.array(low), np.array(high))
            mask2 = cv2.inRange(hsv, np.array([160, min_s, min_v]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if 400 < cv2.contourArea(cnt) < 30000:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                    
                    color_vis = (0, 0, 255) if name=='orange' else ((0,255,0) if name=='Green' else (255,0,0))
                    cv2.drawContours(frame, [cnt], -1, color_vis, 3)
                    cv2.putText(frame, name, (cx-20, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                    objects.append((name, cx, cy))
                    
    cv2.imshow("Color Mask", combined_mask)
    return objects

def mouse_callback(event, x, y, flags, param):
    global state, calib_points, base_point_pixel, base_pos_world, matrix_pixel_to_world, debug_text
    
    if event == cv2.EVENT_LBUTTONDOWN:
        if state == 0:
            calib_points.append((x, y))
            labels =["左下", "右下", "右上", "左上"]
            if len(calib_points) <= 4: print(f"点击: {labels[len(calib_points)-1]}")
            if len(calib_points) == 4:
                pts1 = np.float32(calib_points)
                pts2 = np.float32([[0, CALIB_HEIGHT], [0, 0],[CALIB_WIDTH, 0], [CALIB_WIDTH, CALIB_HEIGHT]])
                matrix_pixel_to_world = cv2.getPerspectiveTransform(pts1, pts2)
                state = 1
                print(">>> 标定完成！请点击底座中心")
        elif state == 1:
            vec = np.array([[[x, y]]], dtype=np.float32)
            base_pos_world = cv2.perspectiveTransform(vec, matrix_pixel_to_world)[0][0]
            base_point_pixel = (x, y)
            state = 2
            print(">>> 底座OK！点击物体抓取")
        elif state == 2:
            target = None
            min_dist = 50
            for obj in detected_objects:
                d = math.hypot(obj[1]-x, obj[2]-y)
                if d < min_dist:
                    min_dist = d
                    target = obj
            if target: 
                process_target(target[1], target[2], target[0])
                
    elif event == cv2.EVENT_RBUTTONDOWN:
        if current_frame is not None:
            # 防止点击边缘报错
            try:
                hsv_pix = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)[y, x]
                debug_text = f"HSV: H={hsv_pix[0]} S={hsv_pix[1]} V={hsv_pix[2]}"
                print(f">>> {debug_text}")
            except Exception as e:
                pass

def main():
    # ⭐ 声明所有会在主循环中修改的全局变量
    global detected_objects, current_frame, state, calib_points, base_point_pixel, debug_text, base_pos_world, matrix_pixel_to_world
    
    print(f"连接摄像头: {STREAM_URL}")
    vs = VideoStream(STREAM_URL).start()
    time.sleep(1.0) 

    cv2.namedWindow("Vision", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Vision", mouse_callback)
    setup_trackbars()

    print("=== 系统就绪 ===")
    
    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.1); continue

        try:
            br = cv2.getTrackbarPos("Brightness", "Settings") - 50
            if br != 0: frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=br)
        except: pass

        current_frame = frame.copy()
        detected_objects = detect_colors(frame)
        
        for i, pt in enumerate(calib_points): cv2.circle(frame, pt, 5, (0,0,255), -1)
        if base_point_pixel: cv2.circle(frame, base_point_pixel, 8, (255,0,255), -1)
        
        # ⭐ 修复文字越界：动态获取分辨率，将文本永远绘制在底部
        if debug_text:
            h, w = frame.shape[:2]
            cv2.rectangle(frame, (0, h-40), (w, h), (0,0,0), -1)
            cv2.putText(frame, debug_text, (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            
        msg = "1.Corners" if state==0 else ("2.Base" if state==1 else "3.GRAB MODE")
        cv2.putText(frame, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow("Vision", frame)
        
        key = cv2.waitKey(1)
        if key == ord('q'): break
        if key == ord('r'): 
            # ⭐ 彻底清除所有历史数据，确保重新标定时坐标系完全重置
            state = 0
            calib_points = []
            base_point_pixel = None
            base_pos_world = None
            matrix_pixel_to_world = None
            debug_text = ""
            print("\n>>> 标定数据已重置！请重新点击 A4 纸的 4 个角。")

    # === 退出程序后的清理工作 ===
    print("正在关闭系统...")
    vs.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户强制退出")
    except Exception as e:
        print(f"\n程序运行出错: {e}")