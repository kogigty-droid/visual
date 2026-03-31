# 版本：
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import threading
import time
import queue
import numpy as np
import cv2
from collections import deque
import re

# ==================== 0. 动态路径处理 ====================
current_file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file_path)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

# 稳定性优化
cv2.setNumThreads(1)

# ==================== 1. 导入依赖 ====================
try:
    from rknnlite.api import RKNNLite
except ImportError:
    print("错误: 需要安装 rknn-toolkit-lite2")
    sys.exit(1)

try:
    from astra_camera_msgs.msg import Detection, Detections
except ImportError:
    Detection = None
    Detections = None

try:
    from utils import operators
    from utils.rec_postprocess import CTCLabelDecode
except ImportError as e:
    print(f"Error: 无法导入 utils: {e}")
    sys.exit(1)

# ==================== 2. 配置参数（全部优化） ====================
YOLO_MODEL_PATH = '/home/orangepi/ros2_ws/src/elevator_detect/elevator_detect/yolov8_bu_op_cl.rknn'
OCR_MODEL_PATH  = '/home/orangepi/ros2_ws/src/elevator_detect/elevator_detect/ppocr.rknn' 
KEYS_PATH       = '/home/orangepi/ros2_ws/src/elevator_detect/elevator_detect/digit.txt'

IN_H, IN_W = 640, 640
OBJ_THRESH = 0.85          # 略微降低，防止漏检
NMS_THRESH = 0.35          # 密集按钮专用，减少误合并
IMG_SIZE = (IN_W, IN_H)
REC_INPUT_SHAPE = [48, 160] 

YOLO_POOL_THREADS = 2
FRAME_QUEUE_MAX = 2
RESULT_QUEUE_MAX = 3

# OCR & 稳定性参数（核心优化）
OCR_CONF_THRESH = 0.85       
OCR_REFRESH_INTERVAL = 1.2   # 延长OCR刷新间隔
EMA_ALPHA = 0.1              
STABLE_WINDOW_SIZE = 10      
STABLE_THRESHOLD = 0.03      # 3cm波动阈值，不再跳变
REGISTRY_EXPIRY = 3.0        # 目标消失后保留3秒

# ==================== 3. 基础组件 ====================

def nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float):
    if boxes.size == 0: 
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1: 
            break
        xx1, yy1 = np.maximum(x1[i], x1[order[1:]]), np.maximum(y1[i], y2[order[1:]])
        xx2, yy2 = np.minimum(x2[i], x2[order[1:]]), np.minimum(y2[i], y2[order[1:]])
        w, h = np.maximum(0.0, xx2 - xx1), np.maximum(0.0, yy2 - yy1)
        iou = (w * h) / (areas[i] + areas[order[1:]] - (w * h) + 1e-6)
        order = order[1:][iou <= iou_thresh]
    return keep

class DetectBox:
    __slots__ = ('classId', 'score', 'xmin', 'ymin', 'xmax', 'ymax', 'track_id')
    def __init__(self, classId, score, xmin, ymin, xmax, ymax):
        self.classId, self.score = int(classId), float(score)
        self.xmin, self.ymin, self.xmax, self.ymax = xmin, ymin, xmax, ymax
        self.track_id = -1

# ==================== 优化版追踪器（卡尔曼预测 + 强容错） ====================
class IDTracker:
    def __init__(self, max_age_sec=1.2, iou_thresh=0.25, alpha=0.7):
        self.max_age_sec = max_age_sec
        self.iou_thresh = iou_thresh
        self.alpha = alpha
        self.tracks = []
        self.next_id = 0

    def _kalman_filter_predict(self, track):
        if 'prev_pos' not in track:
            track['prev_pos'] = (track['box'].xmin, track['box'].ymin, track['box'].xmax, track['box'].ymax)
            return track['box']
        
        prev_x1, prev_y1, prev_x2, prev_y2 = track['prev_pos']
        curr_x1, curr_y1, curr_x2, curr_y2 = track['box'].xmin, track['box'].ymin, track['box'].xmax, track['box'].ymax
        
        pred_x1 = curr_x1 * 0.95 + prev_x1 * 0.05
        pred_y1 = curr_y1 * 0.95 + prev_y1 * 0.05
        pred_x2 = curr_x2 * 0.95 + prev_x2 * 0.05
        pred_y2 = curr_y2 * 0.95 + prev_y2 * 0.05
        
        track['prev_pos'] = (curr_x1, curr_y1, curr_x2, curr_y2)
        return DetectBox(track['box'].classId, track['box'].score, pred_x1, pred_y1, pred_x2, pred_y2)

    def update(self, detections, now_ts):
        used = set()
        new_tracks = []
        for tr in self.tracks:
            tbox = tr['box']
            best_iou, best_j = 0.0, -1
            pred_box = self._kalman_filter_predict(tr)
            
            for j, det in enumerate(detections):
                if j in used or det.classId != pred_box.classId:
                    continue
                iou = self._iou(pred_box, det)
                if iou > best_iou:
                    best_iou = iou
                    best_j = j
                    
            if best_iou >= self.iou_thresh and best_j >= 0:
                det = detections[best_j]
                used.add(best_j)
                det.track_id = tbox.track_id
                a = self.alpha
                det.xmin = tbox.xmin*(1-a) + det.xmin*a
                det.ymin = tbox.ymin*(1-a) + det.ymin*a
                det.xmax = tbox.xmax*(1-a) + det.xmax*a
                det.ymax = tbox.ymax*(1-a) + det.ymax*a
                new_tracks.append({'box': det, 'last_seen': now_ts})
            elif (now_ts - tr['last_seen']) <= self.max_age_sec:
                new_tracks.append(tr)
                
        for j, det in enumerate(detections):
            if j not in used:
                det.track_id = self.next_id
                self.next_id += 1
                new_tracks.append({'box': det, 'last_seen': now_ts})
                
        self.tracks = new_tracks
        return [t['box'] for t in self.tracks]
    
    def _iou(self, b1, b2):
        xx1, yy1 = max(b1.xmin, b2.xmin), max(b1.ymin, b2.ymin)
        xx2, yy2 = min(b1.xmax, b2.xmax), min(b1.ymax, b2.ymax)
        w, h = max(0.0, xx2-xx1), max(0.0, yy2-yy1)
        inter = w * h
        a1 = (b1.xmax-b1.xmin)*(b1.ymax-b1.ymin)
        a2 = (b2.xmax-b2.xmin)*(b2.ymax-b2.ymin)
        return inter / (a1 + a2 - inter + 1e-6)

class RKNNInferencePool:
    def __init__(self, model_path, num_threads=2):
        self.rknn_list = []
        self.free_indices = queue.Queue()
        for i in range(num_threads):
            rknn = RKNNLite(verbose=False)
            rknn.load_rknn(model_path)
            core = RKNNLite.NPU_CORE_0 if i % 2 == 0 else RKNNLite.NPU_CORE_1
            rknn.init_runtime(core_mask=core)
            self.rknn_list.append(rknn)
            self.free_indices.put(i)
            
    def get_rknn_index(self):
        return self.free_indices.get()
    
    def release_rknn_index(self, idx):
        self.free_indices.put(idx)
        
    def inference(self, idx, img):
        return self.rknn_list[idx].inference(inputs=[img], data_format='nhwc')

class OptimizedPostProcess:
    def __init__(self):
        self.strides = [8, 16, 32]
        self.grid = [self._make_grid(s) for s in self.strides]
        
    def _make_grid(self, s):
        h, w = IN_H // s, IN_W // s
        y, x = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        return (np.stack([x, y], axis=-1).reshape(-1, 2) + 0.5) * s
    
    def process(self, outputs, img_h, img_w, thresh):
        all_boxes, all_scores, all_cls = [], [], []
        scale_w, scale_h = img_w / IN_W, img_h / IN_H
        for i, s in enumerate(self.strides):
            reg, cls = outputs[i*2], outputs[i*2+1]
            if reg.ndim == 4:
                reg = reg.transpose(0, 2, 3, 1).reshape(-1, 64)
            if cls.ndim == 4:
                cls = cls.transpose(0, 2, 3, 1).reshape(-1, cls.shape[1])
            cls_scores = 1 / (1 + np.exp(-np.clip(cls, -15, 15)))
            conf = np.max(cls_scores, axis=1)
            mask = conf > thresh
            if not np.any(mask):
                continue
            v = reg[mask].reshape(-1, 4, 16)
            p = np.exp(v) / np.sum(np.exp(v), axis=2, keepdims=True)
            dist = np.sum(p * np.arange(16), axis=2) * s
            g = self.grid[i][mask]
            x1, y1 = (g[:, 0] - dist[:, 0]) * scale_w, (g[:, 1] - dist[:, 1]) * scale_h
            x2, y2 = (g[:, 0] + dist[:, 2]) * scale_w, (g[:, 1] + dist[:, 3]) * scale_h
            all_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
            all_scores.append(conf[mask])
            all_cls.append(np.argmax(cls_scores[mask], axis=1))
            
        if not all_boxes:
            return []
        boxes = np.concatenate(all_boxes)
        scores = np.concatenate(all_scores)
        clss = np.concatenate(all_cls)
        keep = nms_xyxy(boxes, scores, NMS_THRESH)
        return [DetectBox(clss[k], scores[k], *boxes[k]) for k in keep]

# ==================== 4. OCR 引擎 ====================

def resize_with_padding(img, input_shape):
    target_h, target_w = input_shape
    h, w = img.shape[:2]
    scale = target_h / max(1, h)
    new_w = min(int(w * scale), target_w)
    img_resized = cv2.resize(img, (new_w, target_h))
    pad = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    pad[:, :new_w, :] = img_resized
    return pad

class OCRRecognizer:
    def __init__(self, model_path, keys_path):
        self.rknn = RKNNLite(verbose=False)
        self.rknn.load_rknn(model_path)
        self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
        self.postprocess_op = CTCLabelDecode(character_dict_path=keys_path, use_space_char=True)
        
    def recognize(self, img_crop):
        input_tensor = resize_with_padding(img_crop, REC_INPUT_SHAPE)[np.newaxis, :]
        outputs = self.rknn.inference(inputs=[input_tensor], data_format='nhwc')
        res = self.postprocess_op(outputs[0])
        return (res[0][0], float(res[0][1])) if res else ("", 0.0)

OCR_CACHE = {}
OCR_CACHE_LOCK = threading.Lock()
OCR_PENDING = set()
OCR_PENDING_LOCK = threading.Lock()

class AsyncOCRWorker(threading.Thread):
    def __init__(self, model_path, keys_path):
        super().__init__(daemon=True)
        self.ocr_engine = OCRRecognizer(model_path, keys_path)
        self.task_queue = queue.Queue(maxsize=10)
        self.running = True
        
    def run(self):
        print("✅ OCR 引擎后台就绪")
        while self.running:
            tid = None
            try:
                task = self.task_queue.get(timeout=0.1)
                tid, roi = task
                text, conf = self.ocr_engine.recognize(roi)
                if text and conf > OCR_CONF_THRESH:
                    with OCR_CACHE_LOCK:
                        OCR_CACHE[tid] = {'text': text, 'conf': conf, 'ts': time.time()}
            except queue.Empty:
                continue
            except Exception as e:
                print(f"OCR Error: {e}")
            finally:
                if tid is not None:
                    with OCR_PENDING_LOCK:
                        OCR_PENDING.discard(tid)
                    self.task_queue.task_done()
                    
    def add_task(self, tid, roi):
        with OCR_PENDING_LOCK:
            if tid in OCR_PENDING:
                return False
            OCR_PENDING.add(tid)
        try:
            self.task_queue.put_nowait((tid, roi))
            return True
        except:
            with OCR_PENDING_LOCK:
                OCR_PENDING.discard(tid)
            return False

# ==================== 5. ROS2 主节点 ====================

class Elevator3DDetectorNode(Node):
    def __init__(self):
        super().__init__('elevator_3d_detector_node')
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.intrinsics_ready = False
        self.latest_depth_img = None
        self.depth_lock = threading.Lock()

        self.stable_registry = {}
        self.global_button_db = {} 

        self.last_dashboard_ts = 0

        self.yolo_pool = RKNNInferencePool(YOLO_MODEL_PATH, num_threads=2)
        self.yolo_post = OptimizedPostProcess()
        self.tracker = IDTracker(max_age_sec=1.2, iou_thresh=0.25, alpha=0.7)
        self.ocr_worker = AsyncOCRWorker(OCR_MODEL_PATH, KEYS_PATH)
        self.ocr_worker.start()

        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(Detections, '/detection/detections_3d', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 1)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_MAX)
        self.result_queue = queue.Queue(maxsize=RESULT_QUEUE_MAX)
        self.running = True
        threading.Thread(target=self.inference_worker, daemon=True).start()
        threading.Thread(target=self.display_worker, daemon=True).start()
        self.get_logger().info("✅ 稳定增强版启动：彻底解决跳变、消失问题！")

    def info_callback(self, msg):
        if not self.intrinsics_ready:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            self.intrinsics_ready = True
            print(f"✅ 相机内参加载成功")

    def depth_callback(self, msg):
        try:
            with self.depth_lock:
                self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except:
            pass

    def color_callback(self, msg):
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except:
                pass
        try:
            self.frame_queue.put_nowait(self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        except:
            pass

    def get_3d_point(self, u, v):
        if not self.intrinsics_ready or self.latest_depth_img is None:
            return None
        with self.depth_lock:
            u, v = int(u), int(v)
            h, w = self.latest_depth_img.shape
            if not (2 <= u < w-2 and 2 <= v < h-2):
                return None
            roi = self.latest_depth_img[v-2:v+3, u-2:u+3]
            valid = roi[roi > 0]
            if len(valid) == 0:
                return None
            z_m = np.median(valid) / 1000.0
            return [(u-self.cx)*z_m/self.fx, (v-self.cy)*z_m/self.fy, z_m]

    def inference_worker(self):
        while self.running:
            try:
                img = self.frame_queue.get(timeout=0.1)
                blob = np.expand_dims(cv2.cvtColor(cv2.resize(img, IMG_SIZE), cv2.COLOR_BGR2RGB), 0).astype(np.uint8)
                idx = self.yolo_pool.get_rknn_index()
                outs = self.yolo_pool.inference(idx, blob)
                self.yolo_pool.release_rknn_index(idx)
                boxes = self.yolo_post.process(outs, img.shape[0], img.shape[1], OBJ_THRESH)
                if self.result_queue.full():
                    self.result_queue.get_nowait()
                self.result_queue.put_nowait((img, boxes))
            except queue.Empty:
                continue

    def display_worker(self):
        cv2.namedWindow("3D_Stable_Final", cv2.WINDOW_NORMAL)
        while self.running:
            try:
                frame, raw_boxes = self.result_queue.get(timeout=0.1)
                now = time.time()
                stable_boxes = self.tracker.update(raw_boxes, now)

                self.global_button_db = {k: v for k, v in self.global_button_db.items() if (now - v['ts']) < REGISTRY_EXPIRY}
                active_ids = [b.track_id for b in stable_boxes]
                self.stable_registry = {k: v for k, v in self.stable_registry.items() if k in active_ids}

                for b in stable_boxes:
                    with OCR_CACHE_LOCK:
                        cached = OCR_CACHE.get(b.track_id)

                    # ==================== OCR防跳变核心逻辑 ====================
                    if (b.track_id not in OCR_CACHE or 
                        (now - OCR_CACHE.get(b.track_id, {'ts':0})['ts'] > OCR_REFRESH_INTERVAL) or
                        OCR_CACHE.get(b.track_id, {'text':'?'})['text'] == "?") and b.classId == 0:
                        
                        x1, y1 = int(max(0,b.xmin)), int(max(0,b.ymin))
                        x2, y2 = int(min(frame.shape[1],b.xmax)), int(min(frame.shape[0],b.ymax))
                        px, py = int((x2-x1)*0.15), int((y2-y1)*0.15)
                        if (x2-x1)>10 and (y2-y1)>10:
                            self.ocr_worker.add_task(b.track_id, frame[y1+py:y2-py, x1+px:x2-px].copy())

                    # 3D坐标平滑
                    xyz_raw = self.get_3d_point((b.xmin+b.xmax)/2, (b.ymin+b.ymax)/2)
                    if b.track_id not in self.stable_registry:
                        self.stable_registry[b.track_id] = {"xyz": [0,0,0], "buffer": deque(maxlen=STABLE_WINDOW_SIZE)}

                    entry = self.stable_registry[b.track_id]
                    if xyz_raw:
                        if entry["xyz"] == [0,0,0]:
                            entry["xyz"] = xyz_raw
                        else:
                            entry["xyz"] = [entry["xyz"][i]*(1-EMA_ALPHA) + xyz_raw[i]*EMA_ALPHA for i in range(3)]
                        entry["buffer"].append(xyz_raw)

                    # 数字持久化
                    if cached:
                        digit = cached["text"]
                        self.global_button_db[digit] = {
                            "xyz": entry["xyz"],
                            "ts": now,
                            "locked": (len(entry["buffer"]) == STABLE_WINDOW_SIZE and
                                      np.all(np.max(entry["buffer"], axis=0) - np.min(entry["buffer"], axis=0) < STABLE_THRESHOLD))
                        }

                    # 显示
                    current_digit = "?"
                    locked = False
                    for d_key, d_val in self.global_button_db.items():
                        if cached and d_key == cached["text"]:
                            current_digit = d_key
                            locked = d_val["locked"]

                    clr = (0, 255, 0) if locked else (0, 165, 255)
                    cv2.rectangle(frame, (int(b.xmin), int(b.ymin)), (int(b.xmax), int(b.ymax)), clr, 2)
                    label = f"ID{b.track_id} {current_digit} {entry['xyz'][2]:.2f}m"
                    if locked: label = "[OK] " + label
                    cv2.putText(frame, label, (int(b.xmin), int(b.ymin)-10), 1, 1.2, clr, 2)

                if now - self.last_dashboard_ts > 0.5:
                    self.print_dashboard()
                    self.last_dashboard_ts = now

                cv2.imshow("3D_Stable_Final", frame)
                if cv2.waitKey(1) == ord('q'):
                    self.running = False
                    break
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Display Error: {e}")

    def natural_sort_key(self, s):
        return [int(text) if text.isdigit() else text.lower() for text in re.split('([0-9]+)', s)]

    def print_dashboard(self):
        print("\033[H\033[J", end="")
        print("="*65)
        print(f"{'按钮':^8} | {'状态':^10} | {'X (m)':^10} | {'Y (m)':^10} | {'Z距离 (m)':^10}")
        print("-"*65)
        sorted_digits = sorted(self.global_button_db.keys(), key=self.natural_sort_key)
        for digit in sorted_digits:
            it = self.global_button_db[digit]
            state = "LOCKED" if it["locked"] else "STABILIZING"
            print(f"{digit:^8} | {state:^10} | {it['xyz'][0]:10.3f} | {it['xyz'][1]:10.3f} | {it['xyz'][2]:10.3f}")
        print("="*65)

def main():
    rclpy.init()
    node = Elevator3DDetectorNode()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.running = False
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
