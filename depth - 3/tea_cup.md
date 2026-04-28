# 泡茶机器人
## yolo_cup.py:

```python

# 4️⃣ 标定逻辑 (修改为每次强制运行)
# ==============================
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

# ✅ 每次启动直接运行标定
if USE_CALIB:
    H = calibrate()
else:
    H = None

```
