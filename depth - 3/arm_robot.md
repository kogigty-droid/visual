# ArUco 视觉定位快速上手教程

## 1. 本教程能帮你完成什么

按照本教程操作，你可以从零开始搭建一个 ArUco 视觉定位程序，并成功运行摄像头识别。

最终效果：

- 摄像头可以识别 ArUco 标记
- 屏幕上可以看到标记位置和运动轨迹
- 程序会自动生成 `Haoge_data.json`
- 移动 ArUco 标记时，JSON 文件中的 `z`、`y`、`theta` 会实时变化

这份教程不重点讲复杂原理，目标是让你能快速搭建并成功使用。

## 2. 准备材料

你需要准备：

- 一台电脑
- 一个普通 USB 摄像头，或电脑自带摄像头
- Python 环境
- 产品随附的 ArUco 标记
- 项目代码文件 `aruco_gpos_2.py`

建议使用：

- Python 3.8 或更高版本
- OpenCV contrib 版本

## 3. 安装 Python

先确认电脑上是否已经安装 Python。

打开终端，输入：

```bash
python --version
```

或者：

```bash
python3 --version
```

如果能看到类似下面的内容，说明 Python 已安装：

```text
Python 3.8.10
```

如果没有安装 Python，请先到 Python 官网下载安装。

## 4. 安装依赖库

本项目需要安装两个库：

```bash
pip install opencv-contrib-python numpy
```

如果你的电脑使用的是 `python3`，可以使用：

```bash
python3 -m pip install opencv-contrib-python numpy
```

安装完成后，测试 OpenCV 是否可用：

```bash
python
```

进入 Python 后输入：

```python
import cv2
print(cv2.__version__)
print(hasattr(cv2, "aruco"))
```

如果最后一行输出：

```text
True
```

说明安装正确。

如果输出 `False`，请重新安装：

```bash
pip uninstall opencv-python opencv-contrib-python
pip install opencv-contrib-python numpy
```

## 5. 准备 ArUco 标记

本产品会随附已经制作好的 ArUco 标记，用户不需要自行生成、打印或粘贴。

收到产品后，请确认：

- ArUco 标记表面清晰
- 标记没有明显弯折、破损或强反光
- 标记能完整出现在摄像头画面中
- 标记类型与程序默认设置匹配

## 6. 测量 Marker 尺寸

通常情况下，产品随附的 ArUco 标记尺寸已经和程序默认参数匹配。

如果产品资料中已经标注了 ArUco 边长，请直接使用资料中的尺寸。

如果需要自行确认，可以用尺子测量 ArUco 黑色外框的实际边长。

例如实际边长是 2.8 cm，那么程序中应设置：

```python
MARKER_SIZE = 0.028
```

单位是米。

常见换算：

```text
2.8 cm = 0.028 m
3.0 cm = 0.030 m
5.0 cm = 0.050 m
```

这个值一定要尽量准确，否则距离计算会不准。

## 7. 测试摄像头

运行主程序前，建议先确认摄像头可以正常打开。

新建：

```text
camera_test.py
```

写入：

```python
import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    cv2.imshow("camera test", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
```

运行：

```bash
python camera_test.py
```

如果能看到摄像头画面，说明摄像头正常。

如果打不开，把：

```python
cap = cv2.VideoCapture(0)
```

改成：

```python
cap = cv2.VideoCapture(1)
```

或者：

```python
cap = cv2.VideoCapture(2)
```

## 8. 修改主程序参数

打开 `aruco_gpos_2.py`。

首先确认 marker 尺寸：

```python
MARKER_SIZE = 0.028
```

把它改成产品资料中标注的尺寸，或者你实测得到的尺寸。

然后确认摄像头编号：

```python
cap = cv2.VideoCapture(0)
```

如果你在摄像头测试时使用的是 `1`，这里也要改成：

```python
cap = cv2.VideoCapture(1)
```

## 9. 运行主程序

进入代码所在目录。

运行：

```bash
python aruco_gpos_2.py
```

或者：

```bash
python3 aruco_gpos_2.py
```

程序启动后，会弹出两个窗口：

- `Industrial ArUco Tracking - Z + Y + Theta`
- `3D Trajectory`

第一个窗口是摄像头画面。第二个窗口是轨迹显示。

## 10. 正确使用步骤

请按照下面步骤操作：

1. 运行程序。
2. 把 ArUco 标记放到摄像头前。
3. 让标记完整出现在画面中。
4. 把标记放在画面中心附近。
5. 保持不动约 2 秒。
6. 终端出现“开始记录控制原点”后，就可以移动标记。
7. 前后移动标记，观察 `z` 变化。
8. 上下移动标记，观察 `y` 变化。
9. 左右移动标记，观察 `theta` 变化。
10. 按键盘 `Esc` 退出程序。

程序运行后，会在当前目录生成：

```text
Haoge_data.json
```

里面的数据类似：

```json
{
  "z": 0.1111,
  "y": 0.0983,
  "theta": 0.0,
  "pitch": 0.0,
  "yaw": -1.57
}
```

当你移动 ArUco 标记时，里面的 `z`、`y`、`theta` 会不断更新。

## 11. 屏幕信息怎么看

摄像头窗口中会显示几行信息。

```text
Vision X Y Z
```

表示当前识别到的 ArUco 位置。

```text
dx dy dz
```

表示相对开始位置的移动量。

```text
Robot z y theta
```

表示程序输出给后续控制程序的数据。

终端中也会持续打印类似内容：

```text
Vision | X:-0.0123 Y:0.0210 Z:-0.3560 | dx:0.0100 dy:0.0050 dz:-0.0200 || z_cmd:0.0811 y_cmd:0.1021 theta_cmd:-0.0600
```

只要这些数值会随着标记移动而变化，说明视觉部分已经正常工作。

## 12. 控制方向说明

当前程序默认对应关系：

```text
ArUco 前后移动 -> z
ArUco 上下移动 -> y
ArUco 左右移动 -> theta
```

如果你发现方向反了，可以修改下面几个参数。

前后方向反了：

```python
Z_DIRECTION = -1.0
```

上下方向反了：

```python
Y_DIRECTION = -1.0
```

左右旋转方向反了：

```python
THETA_DIRECTION = 1.0
```

修改后重新运行程序。

## 13. 灵敏度调节

如果移动一点点，输出变化太大，说明太灵敏。

可以减小：

```python
K_Z = 1.5
K_Y = 0.75
K_THETA = 6.0
```

如果移动很多，输出变化很小，说明不够灵敏。

可以增大：

```python
K_Z
K_Y
K_THETA
```

建议每次只改一个参数，改完重新运行测试。

## 14. 抖动调节

如果 ArUco 没怎么动，但是输出一直跳，可以适当增大死区参数：

```python
DEADZONE_Z = 0.004
DEADZONE_Y = 0.003
DEADZONE_THETA_X = 0.003
```

例如可以改成：

```python
DEADZONE_Z = 0.006
DEADZONE_Y = 0.005
DEADZONE_THETA_X = 0.005
```

如果输出变化太突然，可以减小单步变化限制：

```python
MAX_STEP_Z = 0.03
MAX_STEP_Y = 0.020
MAX_STEP_THETA = 0.20
```

## 15. 常见问题

### 15.1 程序运行后没有画面

检查摄像头编号。

把：

```python
cap = cv2.VideoCapture(0)
```

改成：

```python
cap = cv2.VideoCapture(1)
```

或：

```python
cap = cv2.VideoCapture(2)
```

### 15.2 摄像头有画面，但识别不到 ArUco

检查以下内容：

- ArUco 是否完整出现在画面中
- 光线是否太暗
- ArUco 表面是否清晰
- 摄像头是否对焦
- ArUco 类型是否与程序默认的 `DICT_4X4_50` 匹配
- 是否安装了 `opencv-contrib-python`

### 15.3 距离明显不准

优先检查：

```python
MARKER_SIZE = 0.028
```

这个值必须等于 ArUco 黑色外框的实际边长，单位是米。

### 15.4 JSON 文件没有变化

可能原因：

- 没有识别到 ArUco
- 还没有保持 2 秒完成原点设置
- 查看错了目录
- 程序没有写文件权限

### 15.5 程序提示没有 cv2.aruco

说明 OpenCV 版本不对。

重新安装：

```bash
pip uninstall opencv-python opencv-contrib-python
pip install opencv-contrib-python numpy
```

## 16. 推荐学习顺序

建议学习者按下面顺序完成：

1. 安装 Python。
2. 安装依赖库。
3. 确认产品随附的 ArUco 标记完整清晰。
4. 测试摄像头。
5. 确认或修改 `MARKER_SIZE`。
6. 运行 `aruco_gpos_2.py`。
7. 保持 ArUco 2 秒设置原点。
8. 移动 ArUco 观察画面和终端输出。
9. 打开 `Haoge_data.json` 查看数据变化。
10. 根据需要调整方向和灵敏度。

## 17. 最终检查清单

运行成功时，你应该看到：

- 摄像头窗口正常显示
- ArUco 周围出现识别框
- 终端持续打印坐标信息
- `Haoge_data.json` 自动生成
- 移动 ArUco 时，`z`、`y`、`theta` 会变化
- 按 `Esc` 可以正常退出

只要以上项目都正常，说明视觉部分已经搭建完成，可以继续对接仿真或机械臂控制程序。
