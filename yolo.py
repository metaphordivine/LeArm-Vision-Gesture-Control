import cv2
import numpy as np
import math
import requests
import time
from ultralytics import YOLO  # 导入 YOLO

# ================= 配置 IP =================
EYE_IP = "192.168.43.8"    # ESP32-CAM IP
ARM_IP = "192.168.43.10"    # ESP32 机械臂 IP
STREAM_URL = f"http://{EYE_IP}:81/stream"

http_session = requests.Session()

# ================= 机械臂环境参数 =================
TABLE_Z = 3.0   # 抓取高度
HOVER_Z = 19.0   # 悬停安全高度

# ⭐ 标定矩形真实尺寸 (A4纸竖放)
CALIB_WIDTH = 21.0  # X轴方向长度
CALIB_HEIGHT = 29.7 # Y轴方向长度

# ================= YOLO 配置 =================
# 加载模型 (首次运行会自动下载 yolov8n.pt)
print("正在加载 YOLO 模型...")
model = YOLO('yolov8n.pt') 

# ⭐ 定义你要抓取的物体及其投放位置
# 格式: 'YOLO类别名': (投放X, 投放Y)
# 常见COCO类别: cup, mouse, bottle, cell phone, apple, orange, scissors...
DROP_POSITIONS = {
    'banana':        (10.0, 15.0),   # 香蕉放左边
    'mouse':      (10.0, -15.0),  # 鼠标放右边
    'bottle':     (20.0, 0.0),    # 瓶子放前面
    'cell phone': (15.0, 10.0),   # 手机放左前
    'toothbrush':      (15.0, -10.0)   # 苹果放右前
}

# 只允许抓取列表中的物体 (防止抓到你的手 'person')
TARGET_CLASSES = list(DROP_POSITIONS.keys())

# 全局状态
calib_points = []       
base_point_pixel = None 
base_pos_world = None   
matrix_pixel_to_world = None 
state = 0 
detected_objects = [] 
conf_threshold = 0.5 # 默认置信度

# ================= UI 调节界面 =================
def setup_trackbars():
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 300, 100)
    # 只需要一个置信度调节
    cv2.createTrackbar("Confidence", "Settings", 50, 95, lambda x: None)

def get_settings():
    global conf_threshold
    # 获取滑动条数值 (0-100 转为 0.0-1.0)
    conf = cv2.getTrackbarPos("Confidence", "Settings")
    if conf < 10: conf = 10
    conf_threshold = conf / 100.0

# ================= 核心逻辑控制 =================
def send_coords_to_esp32(x, y, z, claw_val, move_time=1000, max_retries=3, retry_delay=0.1):
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
            else: return False

def pixel_to_world(u, v):
    if matrix_pixel_to_world is None: return None
    vec = np.array([[[u, v]]], dtype=np.float32)
    pos_world = cv2.perspectiveTransform(vec, matrix_pixel_to_world)
    return pos_world[0][0]

def execute_grab_sequence(x, y, obj_name):
    CLAW_OPEN, CLAW_CLOSE = 90, 10 
    # 获取投放位置，如果没有定义则默认放回 (15,0)
    drop_x, drop_y = DROP_POSITIONS.get(obj_name, (15.0, 0.0))
    
    print(f"\n--- 开始抓取 [{obj_name}] ---")
    
    def step(action, *args):
        print(f"{action}...")
        if not send_coords_to_esp32(*args): return False
        t = args[4] if len(args)>=5 else 1000
        time.sleep(t/1000.0 + 0.2)
        return True
    
    if not step("1. 悬停", x, y, HOVER_Z, CLAW_OPEN, 1000): return
    if not step("2. 下降", x, y, TABLE_Z, CLAW_OPEN, 800): return
    if not step("3. 抓取", x, y, TABLE_Z, CLAW_CLOSE-4, 500): return
    if not step("4. 提起", x, y, HOVER_Z, CLAW_CLOSE-4, 800): return
    if not step("5. 移动投放", drop_x, drop_y, HOVER_Z, CLAW_CLOSE-4, 1500): return
    if not step("6. 松开", drop_x, drop_y, HOVER_Z, CLAW_OPEN, 500): return
    if not step("7. 归位", 5.0, 0.0, HOVER_Z+8, CLAW_OPEN, 1200): return
    
    print("--- 序列完成 ---\n")

def process_target(u, v, obj_name):
    target_pos = pixel_to_world(u, v)
    rob_x = target_pos[0] - base_pos_world[0]
    rob_y = target_pos[1] - base_pos_world[1]
    
    if rob_x < 5.0:
        print(">>> 目标太近或在后方，拒绝抓取！")
        return
    execute_grab_sequence(rob_x, rob_y, obj_name)

def mouse_callback(event, x, y, flags, param):
    global state, calib_points, base_point_pixel, base_pos_world, matrix_pixel_to_world
    if event == cv2.EVENT_LBUTTONDOWN:
        if state == 0:
            calib_points.append((x, y))
            labels = ["左下", "右下", "右上", "左上"]
            if len(calib_points) <= 4: print(f"点击: {labels[len(calib_points)-1]}")
            if len(calib_points) == 4:
                pts_src = np.float32(calib_points)
                pts_dst = np.float32([[0, CALIB_HEIGHT], [0, 0], [CALIB_WIDTH, 0], [CALIB_WIDTH, CALIB_HEIGHT]])
                matrix_pixel_to_world = cv2.getPerspectiveTransform(pts_src, pts_dst)
                state = 1
                print(">>> 标定完成！点击【机械臂底座中心】。")
        elif state == 1:
            base_point_pixel = (x, y)
            base_pos_world = pixel_to_world(x, y)
            state = 2
            print(">>> 就绪！点击画面上的【检测框中心】抓取。")
        elif state == 2:
            # 查找鼠标点击位置最近的物体
            chosen = None
            min_dist = 50
            for obj_name, cx, cy in detected_objects:
                dist = math.hypot(cx - x, cy - y)
                if dist < min_dist: 
                    chosen = (obj_name, cx, cy)
                    min_dist = dist
            if chosen: process_target(chosen[1], chosen[2], chosen[0])

# ⭐ YOLO 识别函数
def detect_objects_yolo(frame):
    # 使用 YOLO 进行预测
    results = model.predict(frame, conf=conf_threshold, verbose=False)
    
    objects = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # 获取类别ID和名称
            cls_id = int(box.cls[0])
            name = model.names[cls_id]
            
            # 只识别我们在 DROP_POSITIONS 里定义的物体
            if name in TARGET_CLASSES:
                # 获取坐标
                x1, y1, x2, y2 = box.xyxy[0]
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                
                # 绘制
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"{name} {box.conf[0]:.2f}", (int(x1), int(y1)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                objects.append((name, cx, cy))
            else:
                # (可选) 绘制其他物体但用灰色显示
                x1, y1, x2, y2 = box.xyxy[0]
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (100, 100, 100), 1)
                cv2.putText(frame, name, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

    return objects

def main():
    global detected_objects, calib_points, base_point_pixel, state
    
    cap = cv2.VideoCapture(STREAM_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
    
    cv2.namedWindow("Vision", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Vision", mouse_callback)
    
    setup_trackbars() # 启动置信度调节

    print("=== LeArm YOLO 视觉抓取系统启动 ===")
    print("步骤1: 依次点击 A4纸的 左下角 -> 右下角 -> 右上角 -> 左上角")

    while True:
        ret, frame = cap.read()
        if not ret: 
            time.sleep(1)
            cap = cv2.VideoCapture(STREAM_URL)
            continue
        
        # 获取最新的置信度设置
        get_settings()

        # ⭐ 使用 YOLO 进行识别
        detected_objects = detect_objects_yolo(frame)
        
        # UI 绘制
        for i, pt in enumerate(calib_points): 
            cv2.circle(frame, pt, 5, (0,0,255), -1)
            cv2.putText(frame, str(i+1), (pt[0]+10, pt[1]-10), 1, 1, (0,0,255), 2)
        if base_point_pixel: 
            cv2.circle(frame, base_point_pixel, 8, (255,0,255), -1)
            cv2.putText(frame, "Base", (base_point_pixel[0]+10, base_point_pixel[1]-10), 1, 1, (255,0,255), 2)
        
        msg = "1. Click A4 Corners" if state==0 else ("2. Click Base Center" if state==1 else "3. Click Object to GRAB")
        cv2.rectangle(frame, (0, 0), (600, 40), (0,0,0), -1)
        cv2.putText(frame, msg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
        
        cv2.imshow("Vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        if cv2.waitKey(1) & 0xFF == ord('r'): 
            state=0; calib_points=[]; base_point_pixel=None
            print(">>> 标定已重置！")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()