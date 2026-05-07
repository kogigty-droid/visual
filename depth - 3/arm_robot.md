# ArUco 视觉定位学习教程

## 1. 教程目标

本教程面向初学者，介绍如何从零搭建基于 OpenCV ArUco 的视觉定位系统。学习完成后，用户可以使用普通摄像头识别 ArUco 标记，计算标记在摄像头坐标系下的三维位置，并将位置变化转换为后续机械臂控制程序可以读取的数据。

本教程重点讲解视觉部分，包括：

- Python 和 OpenCV 环境搭建
- ArUco 标记生成与打印
- 摄像头画面采集
- ArUco 标记检测
- 基于 `solvePnP` 的三维位姿估计
- 视觉坐标平滑与滤波
- 视觉坐标到控制数据的输出
- 程序运行与调试方法

本项目最终会实时生成一个 JSON 文件：

```json
{
  "z": 0.1111,
  "y": 0.0983,
  "theta": 0.0,
  "pitch": 0.0,
  "yaw": -1.57
}
```

这个文件可以被仿真程序或机械臂控制程序读取。

## 2. 系统原理

本项目的基本流程如下：

```text
摄像头采集画面
        ↓
识别 ArUco 标记
        ↓
提取标记四个角点
        ↓
根据真实尺寸和相机参数计算三维位置
        ↓
对坐标进行平滑和滤波
        ↓
设置初始位置为控制原点
        ↓
计算相对位移 dx / dy / dz
        ↓
转换成 z / y / theta 控制量
        ↓
写入 Haoge_data.json
```

在当前程序中，ArUco 的移动和控制量的对应关系是：

```text
ArUco 前后移动 -> 输出 z
ArUco 上下移动 -> 输出 y
ArUco 左右移动 -> 输出 theta
```

注意：本教程讲的是视觉定位和数据输出。实际机械臂是否运动，取决于另一个机械臂控制程序是否读取并执行 `Haoge_data.json` 中的数据。

## 3. 环境准备

### 3.1 安装 Python

推荐使用 Python 3.8 或更高版本。

在终端中输入：

```bash
python3 --version
```

如果系统使用的是 Windows，也可以输入：

```bash
python --version
```

确认能看到类似下面的输出：

```text
Python 3.8.x
```

### 3.2 安装依赖库

本项目主要依赖：

- `opencv-contrib-python`
- `numpy`

安装命令：

```bash
pip install opencv-contrib-python numpy
```

如果你的电脑同时安装了多个 Python 版本，可以使用：

```bash
python3 -m pip install opencv-contrib-python numpy
```

### 3.3 验证 OpenCV ArUco 是否可用

新建一个测试文件，输入：

```python
import cv2

print(cv2.__version__)
print(hasattr(cv2, "aruco"))
```

运行后如果看到：

```text
True
```

说明 ArUco 模块可用。

如果输出 `False`，通常是因为安装了普通版 `opencv-python`，需要改装 `opencv-contrib-python`。

## 4. 准备 ArUco 标记

### 4.1 ArUco 是什么

ArUco 是一种黑白方形视觉标记。摄像头识别到它以后，可以通过标记的四个角点计算出标记相对于摄像头的位置和姿态。

本项目使用的 ArUco 字典是：

```python
cv2.aruco.DICT_4X4_50
```

所以生成和打印标记时，也需要使用同一个字典。

### 4.2 生成 ArUco 图片

可以使用下面的脚本生成一个 ID 为 0 的 ArUco 标记：

```python
import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker = cv2.aruco.generateImageMarker(aruco_dict, 0, 400)

cv2.imwrite("marker_0.png", marker)
print("marker_0.png generated")
```

运行后会得到：

```text
marker_0.png
```

将它打印出来，贴在平整的纸板或硬质平面上。

### 4.3 测量 Marker 实际边长

打印完成后，用尺子测量黑色外框的实际边长。

例如测得边长是 2.8 cm，则程序中应设置：

```python
MARKER_SIZE = 0.028
```

这里单位是米。

这个参数非常重要。如果 `MARKER_SIZE` 不准确，计算出来的三维距离也会不准确。

## 5. 摄像头测试

在运行主程序前，建议先测试摄像头是否可以正常打开。

测试代码：

```python
import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Cannot read camera")
        break

    cv2.imshow("camera test", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
```

如果出现摄像头画面，说明摄像头工作正常。

如果打不开摄像头，可以尝试修改摄像头编号：

```python
cap = cv2.VideoCapture(1)
```

或者：

```python
cap = cv2.VideoCapture(2)
```

## 6. 相机参数说明

视觉定位需要相机内参。当前程序使用一组简化相机参数：

```python
K = np.array([[900, 0, 640],
              [0, 900, 360],
              [0, 0, 1]], dtype=np.float64)

dist = np.zeros((5, 1))
```

其中：

- `K` 是相机内参矩阵
- `dist` 是畸变参数
- `640` 和 `360` 是图像中心点，对应 `1280 x 720` 分辨率
- `900` 是近似焦距参数

程序中摄像头分辨率设置为：

```python
cap.set(3, 1280)
cap.set(4, 720)
```

这组参数可以用于入门测试。如果需要更高精度，建议进行相机标定，得到真实的 `camera_matrix` 和 `dist_coeffs`。

## 7. ArUco 检测流程

### 7.1 初始化 ArUco 检测器

程序使用下面的代码初始化 ArUco 检测器：

```python
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
```

这里的 `DICT_4X4_50` 必须和打印出来的 marker 字典一致。

### 7.2 检测每一帧图像

程序不断读取摄像头画面：

```python
ret, frame = cap.read()
```

然后转换成灰度图：

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
```

再检测 ArUco：

```python
corners, ids, _ = detector.detectMarkers(frame)
```

检测成功后：

- `corners` 保存每个 marker 的四个角点
- `ids` 保存每个 marker 的编号

程序会把检测到的 marker 画在画面上：

```python
cv2.aruco.drawDetectedMarkers(frame, corners, ids)
```

## 8. 亚像素角点优化

为了提高定位精度，程序会对检测到的角点进行亚像素优化：

```python
def refine_corners(gray, corners):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    for c in corners:
        cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), criteria)
    return corners
```

普通检测得到的是像素级角点，而亚像素优化可以让角点位置更精细，从而提高三维位置计算稳定性。

## 9. 三维位姿估计

### 9.1 Marker 的真实角点

程序先根据 marker 实际边长构造四个真实角点：

```python
half = MARKER_SIZE / 2
obj_pts = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)
```

这表示 marker 在真实世界中是一个边长为 `MARKER_SIZE` 的正方形。

### 9.2 使用 solvePnP 求解位置

核心代码：

```python
ok, rvec, tvec = cv2.solvePnP(
    obj_pts,
    img_pts,
    K,
    dist,
    flags=cv2.SOLVEPNP_IPPE_SQUARE
)
```

`solvePnP` 的作用是：

```text
已知 marker 真实尺寸
已知 marker 在图像中的四个角点
已知相机内参
        ↓
求出 marker 相对摄像头的位置和姿态
```

其中：

- `rvec` 表示旋转
- `tvec` 表示平移

程序会把旋转向量转换成旋转矩阵：

```python
R, _ = cv2.Rodrigues(rvec)
```

然后得到三维位置：

```python
t = tvec.reshape(3)
```

因为 `MARKER_SIZE` 的单位是米，所以 `t` 的单位也是米。

## 10. 视觉坐标平滑

ArUco 识别会受到光照、角点误差、摄像头噪声影响，直接使用原始坐标会导致数据抖动。

当前程序使用三种方式提高稳定性。

### 10.1 滑动窗口

```python
window = deque(maxlen=WINDOW_SIZE)
```

程序保存最近几帧的位置数据，默认窗口大小是：

```python
WINDOW_SIZE = 8
```

### 10.2 鲁棒均值

```python
def robust_mean(pts):
    pts = np.array(pts)
    center = np.median(pts, axis=0)
    filtered = [p for p in pts if np.linalg.norm(p - center) < MAX_JUMP]
    return np.mean(filtered, axis=0)
```

它会过滤掉明显跳变的异常点，减少偶发误识别造成的位置突变。

### 10.3 卡尔曼滤波

程序使用 `KF3D` 对三维坐标进行滤波：

```python
kf = KF3D()
center = kf.update(raw)
```

卡尔曼滤波的作用是让坐标变化更连续，适合后续控制使用。

## 11. 设置视觉控制原点

程序启动后，不会立即使用当前位置作为控制量，而是要求用户把 ArUco 放在画面中心附近保持约 2 秒。

代码逻辑：

```python
if time.time() - last_detection_time >= 2.0 and origin is None:
    origin = np.array([x, y, z])
```

这个 `origin` 就是控制原点。

之后程序计算当前位置相对于原点的位移：

```python
dx = x - origin[0]
dy = y - origin[1]
dz = z - origin[2]
```

含义如下：

- `dx`：左右移动量
- `dy`：上下移动量
- `dz`：前后移动量

## 12. 视觉数据到控制数据

本项目视觉部分最终会输出三个主要控制量：

```text
z
y
theta
```

### 12.1 前后移动控制 z

对应函数：

```python
map_dz_to_robot_z(dz, last_z_cmd)
```

核心公式：

```python
z_cmd_raw = ARM_Z_INIT + Z_DIRECTION * K_Z * dz_ctrl
```

相关参数：

```python
ARM_Z_INIT = 0.1111
ARM_Z_MIN = 0.0500
ARM_Z_SAFE_MAX = 0.3700
K_Z = 1.5
Z_DIRECTION = 1.0
DEADZONE_Z = 0.004
MAX_STEP_Z = 0.03
```

说明：

- `K_Z` 控制 z 方向灵敏度
- `Z_DIRECTION` 控制方向，方向反了可以改成 `-1.0`
- `DEADZONE_Z` 用于过滤微小抖动
- `ARM_Z_MIN` 和 `ARM_Z_SAFE_MAX` 用于限制安全范围
- `MAX_STEP_Z` 限制每帧最大变化量

### 12.2 上下移动控制 y

对应函数：

```python
map_dy_to_robot_y(dy, last_y_cmd)
```

核心公式：

```python
y_cmd_raw = ARM_Y_INIT + Y_DIRECTION * K_Y * dy_ctrl
```

相关参数：

```python
ARM_Y_INIT = 0.0983
ARM_Y_MIN = 0.0000
ARM_Y_SAFE_MAX = 0.3100
K_Y = 0.75
Y_DIRECTION = 1.0
DEADZONE_Y = 0.003
MAX_STEP_Y = 0.020
```

### 12.3 左右移动控制 theta

对应函数：

```python
map_dx_to_robot_theta(dx, last_theta_cmd)
```

核心公式：

```python
theta_raw = THETA_INIT + THETA_DIRECTION * K_THETA * dx_ctrl
```

相关参数：

```python
THETA_INIT = 0.0000
THETA_MIN = -1.5708
THETA_MAX = 1.5708
K_THETA = 6.0
THETA_DIRECTION = -1.0
DEADZONE_THETA_X = 0.003
MAX_STEP_THETA = 0.20
```

说明：

- `theta` 通常表示机械臂底座旋转角度
- `K_THETA` 越大，左右移动越灵敏
- 如果左右方向反了，可以把 `THETA_DIRECTION` 改成 `1.0`

## 13. 输出 JSON 文件

程序会实时写入：

```text
Haoge_data.json
```

初始化时先写入默认值：

```python
init_json = {
    "z": round(float(ARM_Z_INIT), 4),
    "y": round(float(ARM_Y_INIT), 4),
    "theta": round(float(THETA_INIT), 4),
    "pitch": round(float(FIXED_PITCH_CMD), 4),
    "yaw": round(float(FIXED_YAW_CMD), 4)
}
```

识别并完成原点设置后，程序会不断更新：

```python
json_data = {
    "z": round(float(z_cmd), 4),
    "y": round(float(y_cmd), 4),
    "theta": round(float(theta_cmd), 4),
    "pitch": round(float(FIXED_PITCH_CMD), 4),
    "yaw": round(float(FIXED_YAW_CMD), 4)
}
```

写入文件：

```python
with open("Haoge_data.json", "w", encoding="utf-8") as f:
    json.dump(json_data, f)
```

后续程序只需要读取 `Haoge_data.json`，就可以获得视觉系统输出的数据。

## 14. 运行程序

进入项目目录后运行：

```bash
python3 aruco_gpos_2.py
```

Windows 用户也可以使用：

```bash
python aruco_gpos_2.py
```

运行后会出现两个窗口：

- `Industrial ArUco Tracking - Z + Y + Theta`
- `3D Trajectory`

第一个窗口显示摄像头画面和识别结果。第二个窗口显示三维轨迹。

## 15. 操作步骤

1. 启动程序。
2. 将打印好的 ArUco 标记放在摄像头前。
3. 保持标记清晰可见。
4. 将 ArUco 放在画面中心附近。
5. 保持约 2 秒，程序会自动记录当前位置为原点。
6. 前后移动 ArUco，观察终端中的 `z_cmd`。
7. 上下移动 ArUco，观察终端中的 `y_cmd`。
8. 左右移动 ArUco，观察终端中的 `theta_cmd`。
9. 按 `Esc` 退出程序。

终端中会看到类似输出：

```text
Vision | X:-0.0123 Y:0.0210 Z:-0.3560 | dx:0.0100 dy:0.0050 dz:-0.0200 || Z_MAP ... y_cmd:0.1021 ... theta_cmd:-0.0600
```

## 16. 屏幕显示信息说明

摄像头窗口会显示：

```text
Vision X / Y / Z
```

表示当前识别到的 ArUco 三维位置。

```text
dx / dy / dz
```

表示相对于原点的移动量。

```text
Robot z / y / theta
```

表示转换后的控制输出。

```text
K_Z / K_Y / K_THETA
```

表示当前使用的灵敏度参数。

## 17. 参数调节建议

### 17.1 Marker 尺寸

如果测量距离明显不准，优先检查：

```python
MARKER_SIZE = 0.028
```

它必须等于打印后实际黑色外框边长，单位是米。

### 17.2 控制方向反了

如果前后方向反了，修改：

```python
Z_DIRECTION = -1.0
```

如果上下方向反了，修改：

```python
Y_DIRECTION = -1.0
```

如果左右旋转方向反了，修改：

```python
THETA_DIRECTION = 1.0
```

### 17.3 控制太灵敏

减小下面的参数：

```python
K_Z = 1.5
K_Y = 0.75
K_THETA = 6.0
```

### 17.4 控制太迟钝

增大下面的参数：

```python
K_Z
K_Y
K_THETA
```

### 17.5 数据抖动明显

可以适当增大死区：

```python
DEADZONE_Z = 0.004
DEADZONE_Y = 0.003
DEADZONE_THETA_X = 0.003
```

也可以减小每帧最大变化量：

```python
MAX_STEP_Z = 0.03
MAX_STEP_Y = 0.020
MAX_STEP_THETA = 0.20
```

## 18. 常见问题

### 18.1 摄像头打不开

检查摄像头编号：

```python
cap = cv2.VideoCapture(0)
```

可以尝试改成：

```python
cap = cv2.VideoCapture(1)
```

### 18.2 检测不到 ArUco

请检查：

- 是否安装 `opencv-contrib-python`
- 打印的 marker 是否来自 `DICT_4X4_50`
- marker 是否完整、清晰、无反光
- 摄像头是否对焦
- 光照是否充足
- marker 是否离摄像头太远

### 18.3 距离数值不准确

常见原因：

- `MARKER_SIZE` 填写错误
- 相机内参不是实际标定结果
- 摄像头畸变较大但 `dist` 设置为 0
- marker 打印后尺寸被缩放

### 18.4 画面中有识别框但没有控制输出

程序需要先稳定识别约 2 秒并记录原点。

请保持 ArUco 在画面中心附近，不要快速移动，直到终端提示：

```text
两秒时间到，开始记录控制原点
```

### 18.5 JSON 文件没有变化

请确认：

- 程序是否已经检测到 ArUco
- 是否已经完成原点设置
- 当前目录下是否有写入权限
- 是否正在查看正确目录下的 `Haoge_data.json`

## 19. 学习者实验任务

建议学习者按顺序完成以下实验：

1. 成功打开摄像头。
2. 成功检测 ArUco。
3. 修改 `MARKER_SIZE`，观察距离变化。
4. 前后移动 ArUco，观察 `dz` 和 `z_cmd`。
5. 上下移动 ArUco，观察 `dy` 和 `y_cmd`。
6. 左右移动 ArUco，观察 `dx` 和 `theta_cmd`。
7. 修改 `K_Z / K_Y / K_THETA`，观察灵敏度变化。
8. 修改死区参数，观察抖动变化。
9. 打开 `Haoge_data.json`，观察实时输出数据。

## 20. 总结

本项目完成了一个完整的 ArUco 视觉定位流程：

```text
摄像头采集
ArUco 检测
三维位姿估计
坐标滤波
原点设置
相对位移计算
控制量映射
JSON 数据输出
```

它可以作为机械臂视觉控制、仿真联调、人机交互控制等项目的视觉输入模块。

对于初学者来说，建议先理解三个关键点：

1. ArUco 的四个角点如何被检测出来。
2. `solvePnP` 如何根据角点计算三维位置。
3. 相对位移 `dx / dy / dz` 如何映射成控制量 `theta / y / z`。

理解这三点后，就可以在当前代码基础上继续扩展，例如加入相机标定、多个 marker 融合、UDP 输出、ROS 通信或真实机械臂 SDK 控制。
