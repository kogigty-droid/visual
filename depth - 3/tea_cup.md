# 泡茶机器人
## yolo_cup.py:

```python

import cv2
import numpy as np
import time
import os
from rknnlite.api import RKNNLite

# ==============================
# 1️⃣ 配置参数
# ==============================
RKNN_MODEL = '/mnt/data/rknn_work/best_rknn_model/best-rk3588.rknn'  # 替换为你的模型路径
USE_CALIB = True
SCALE = 1.0
calib_file = "H.npy"

# YOLO 推理参数
OBJ_THRESH = 0.45
NMS_THRESH = 0.45
IMG_SIZE = 640  # 必须与你导出 RKNN 时的尺寸一致

# ==============================
# 2️⃣ YOLOv8/v11 后处理核心函数
# ==============================
def post_process(input_data):
    """
    针对标准 YOLOv8/v11 RKNN 输出的解析。
    注意：这里适配的是单类检测（茶杯）。如果是多类，请调整类别过滤逻辑。
    """
    boxes, scores, classes_conf = [], [], []
    
    # RKNN 输出通常为 [1, 84, 8400] -> (cx, cy, w, h, score1, score2...)
    # 如果你的模型导出时带了特殊的 Head，这里的形状可能不同
    res = input_data[0].squeeze() 
    if res.shape[0] > res.shape[1]:
        res = res.T
        
    for i in range(res.shape[1]):
        row = res[:, i]
        conf = row[4:].max() # 获取最高类别的置信度
        if conf > OBJ_THRESH:
            # 还原坐标 (基于 640x640)
            cx, cy, w, h = row[0:4]
            x1 = (cx - w/2)
            y1 = (cy - h/2)
            boxes.append([x1, y1, w, h])
            scores.append(float(conf))

    indices = cv2.dnn.NMSBoxes(boxes, scores, OBJ_THRESH, NMS_THRESH)
    
    final_boxes = []
    if len(indices) > 0:
        for i in indices.flatten():
            final_boxes.append((boxes[i], scores[i]))
    return final_boxes

# ==============================
# 3️⃣ RKNN 初始化
# ==============================
rknn_lite = RKNNLite()
print('--> 正在加载 RKNN 模型...')
if rknn_lite.load_rknn(RKNN_MODEL) != 0:
    print('❌ 模型加载失败'); exit(1)

# 在 RK3588 上，可以指定核心：NPU_CORE_0, NPU_CORE_1, NPU_CORE_2
if rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) != 0:
    print('❌ NPU 初始化失败'); exit(1)
print('✅ RKNN 模型加载成功，NPU 已就绪')

# ==============================
# 4️⃣ 标定逻辑（修复缩进+语法）
# ==============================
def find_camera_index(max_idx=5):
    for i in [0, 1, 11, 2]: # 11 常为香橙派 MIPI 摄像头
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cap.release()
            return i
    exit("❌ 未找到摄像头")

# 标定函数（修复缩进，独立定义）
def calibrate():
    cap = cv2.VideoCapture(CAM_INDEX)

    if not cap.isOpened():
        print("❌ 标定启动失败：无法打开摄像头")
        return

    points_img = []

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points_img.append([x, y])
            print(f"点 {len(points_img)}/4：({x},{y})")

    cv2.namedWindow("calibration")
    cv2.setMouseCallback("calibration", mouse_callback)

    print("\n" + "="*30)
    print("📢 进入强制标定模式")
    print("请按顺序点击 170mm 正方形的四个角：")
    print("1. 左上  2. 右上  3. 右下  4. 左下")
    print("="*30)

    while len(points_img) < 4:
        ret, frame = cap.read()
        if not ret: break
        for idx, (px, py) in enumerate(points_img):
            cv2.circle(frame, (px, py), 6, (0, 0, 255), -1)
            cv2.putText(frame, str(idx+1), (px+10, py+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow("calibration", frame)
        if cv2.waitKey(1) == 27: break # 按 ESC 退出

    cv2.destroyAllWindows()
    cap.release()

    if len(points_img) == 4:
        pts_world = np.array([[0,0], [170,0], [170,170], [0,170]], dtype=np.float32)
        H_new, _ = cv2.findHomography(np.array(points_img, dtype=np.float32), pts_world)
        np.save(calib_file, H_new)
        print("✅ 标定成功并保存！")
        return H_new
    else:
        print("❌ 标定未完成，程序退出")
        exit(1)

CAM_INDEX = find_camera_index()

# 执行标定
if USE_CALIB:
    print("🆕 开始标定...")
    H = calibrate()
else:
    if os.path.exists(calib_file):
        H = np.load(calib_file)
        print("✅ 加载已保存的标定文件")
    else:
        H = None
        print("⚠️ 未标定，坐标转换将失效")

def pixel_to_world(u, v):
    pt = np.array([u, v, 1.0])
    world = H @ pt
    world /= world[2]
    return world[0] * SCALE, world[1] * SCALE

# ==============================
# 5️⃣ 主检测循环
# ==============================
cap = cv2.VideoCapture(CAM_INDEX)
# 尝试设置高帧率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ALPHA = 0.7
prev_cx, prev_cy = None, None
last_print_time = 0

print("🚀 启动 NPU 加速识别...")

while True:
    ret, frame = cap.read()
    if not ret: break

    # --- 预处理 ---
    # RKNN 需要固定尺寸输入，且通常为 RGB
    img_in = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_in = cv2.resize(img_in, (IMG_SIZE, IMG_SIZE))
    img_in = np.expand_dims(img_in,axis=0)
    
    # --- NPU 推理 ---
    outputs = rknn_lite.inference(inputs=[img_in])
    
    # --- 后处理 ---
    results = post_process(outputs)
    
    best_box = None
    best_conf = 0
    
    # 找到得分最高的框
    for box_data, conf in results:
        if conf > best_conf:
            best_conf = conf
            best_box = box_data

    if best_box is not None:
        # 将坐标从 640 映射回原始画面尺寸
        h_orig, w_orig = frame.shape[:2]
        scale_x, scale_y = w_orig / IMG_SIZE, h_orig / IMG_SIZE
        x, y, w, h = best_box
        x1, y1 = int(x * scale_x), int(y * scale_y)
        x2, y2 = int((x+w) * scale_x), int((y+h) * scale_y)
        
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        # --- 平滑处理 ---
        if prev_cx is None:
            scx, scy = cx, cy
        else:
            scx = int(ALPHA * prev_cx + (1 - ALPHA) * cx)
            scy = int(ALPHA * prev_cy + (1 - ALPHA) * cy)
        prev_cx, prev_cy = scx, scy

        # --- 坐标转换与打印 ---
        X, Y = 0, 0
        if USE_CALIB and H is not None:
            X, Y = pixel_to_world(scx, scy)
            curr_time = time.time()
            if curr_time - last_print_time >= 1:
                print(f"\r实际坐标(mm): X={X:.1f} Y={Y:.1f}  ", end="")
                last_print_time = curr_time

        # --- 绘制 ---
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (scx, scy), 5, (0, 0, 255), -1)
        label = f"X:{X:.1f} Y:{Y:.1f}" if (USE_CALIB and H is not None) else f"{scx},{scy}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("OrangePi 5Plus NPU Detection", frame)
    if cv2.waitKey(1) == 27: break

cap.release()
rknn_lite.release()
cv2.destroyAllWindows()

```
