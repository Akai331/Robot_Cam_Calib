# Robot Camera Calibration Guide

本文档涵盖机器人相机的**内参标定（Intrinsics）**和**外参标定（Extrinsics）**的完整流程。

标定工具路径：`~/Software/planb-robot/RobotCamCalib/`

---

## 目录

- [1. 概念总览](#1-概念总览)
  - [1.1 内参（Intrinsics）](#11-内参intrinsics)
  - [1.2 外参（Extrinsics）](#12-外参extrinsics)
  - [1.3 数学建模](#13-数学建模)
- [2. 前置准备](#2-前置准备)
- [3. 内参标定](#3-内参标定)
  - [3.1 准备棋盘格](#31-准备棋盘格)
  - [3.2 运行内参标定脚本](#32-运行内参标定脚本)
  - [3.3 确认 / 覆盖内参](#33-确认--覆盖内参)
  - [3.4 内参输出格式](#34-内参输出格式)
- [4. 外参标定](#4-外参标定)
  - [4.1 准备 AprilTag](#41-准备-apriltag)
  - [4.2 让机器人进入拖拽模式](#42-让机器人进入拖拽模式)
  - [4.3 配置外参标定脚本](#43-配置外参标定脚本)
  - [4.4 运行外参标定](#44-运行外参标定)
  - [4.5 外参输出格式](#45-外参输出格式)
- [5. 标定结果验证](#5-标定结果验证)
- [6. 常见问题排查](#6-常见问题排查)
- [附录 A: 关键参数速查](#附录-a-关键参数速查)
- [附录 B: 文件目录结构](#附录-b-文件目录结构)

---

## 1. 概念总览

### 1.1 内参（Intrinsics）

内参描述相机自身的光学特性，将三维空间点从**相机坐标系**投影到**像素坐标系**。

内参矩阵 **K**：

```
K = | fx   0   cx |
    |  0  fy   cy |
    |  0   0    1 |
```

| 参数 | 含义 |
|------|------|
| `fx`, `fy` | 焦距（单位：像素），分别对应 x 和 y 方向 |
| `cx`, `cy` | 主点坐标（光心在图像上的投影位置） |

畸变系数 **dist**：`[k1, k2, p1, p2, k3]`

| 参数 | 含义 |
|------|------|
| `k1`, `k2`, `k3` | 径向畸变系数（桶形 / 枕形失真） |
| `p1`, `p2` | 切向畸变系数（镜头与传感器不完全平行） |

### 1.2 外参（Extrinsics）

外参描述相机在机器人坐标系中的位置和姿态。我们使用 **Hand-Eye Calibration** 方法（AX = YB 问题），通过多组观测求解相机安装位姿。

对于我们的系统，需要求解两个 4x4 齐次变换矩阵：

| 矩阵 | 含义 |
|------|------|
| `X_CammountCam` | 相机安装关节 → 相机光心的变换（**我们最终需要的外参**） |
| `X_TagmountTag` | AprilTag安装关节 → AprilTag的变换（辅助求解） |

### 1.3 数学建模

外参标定的核心是求解 **AX = YB** 问题：

```
A_i @ X ≈ Y @ B_i    (对所有第 i 次观测成立)
```

- **A_i** = `inv(X_WorldCammount_i) @ X_WorldTagmount_i` — 由 URDF 正运动学计算（精确已知）
- **B_i** = `X_CamTag_i` — 由 AprilTag 检测获得（含噪声）
- **X** = `X_TagmountTag` — 待求解
- **Y** = `X_CammountCam` — 待求解

求解方法：概率 MLE + Gauss-Newton 迭代优化，在 SE(3) 李群上进行左乘更新，使用 Huber 鲁棒损失函数抑制离群点。

---

## 2. 前置准备

### 环境配置

```bash
# rosci 运行环境
cd ~/rosci_software
source env_rosci_setup.bash
```

### 硬件要求

- Intel RealSense D455/D435 深度相机（已安装在机器人上）
- A4 打印纸 + 平整板子（用于贴标定板/AprilTag）
- 机器人可正常运动、进入拖拽模式

---

## 3. 内参标定

> **注意**：如果已经从相机 SDK 获取了准确的内参（如 RealSense 出厂标定），可以直接使用，跳到 [3.3 确认 / 覆盖内参](#33-确认--覆盖内参)。

### 3.1 准备棋盘格

1. 打印棋盘格图案：
   ```
   文件路径：~/Software/planb-robot/RobotCamCalib/assets/intr_calib_checkerboard.pdf
   ```
2. 打印规格：**A4 纸、100% 缩放**（不要"适合页面"）
3. 将打印出的棋盘格贴在一块**平整硬板**上（避免弯曲）

棋盘格参数（脚本中已配置）：
- 内角点数量：8 x 6
- 方格边长：25mm
> **注意**：如果用别的标定板，记得修改棋盘格参数！！！

### 3.2 运行内参标定脚本

```bash
cd ~/Software/planb-robot/RobotCamCalib
python3 intr_calib.py
```

操作步骤：

1. 脚本会打开相机实时画面（`InteractiveCamera` 界面）
2. 手持棋盘格板在相机前不同位置、不同角度移动
3. 按 **`s`** 键采集当前帧（需要在画面中能清晰看到完整棋盘格）
4. 重复采集，**至少 12 帧有效检测**（建议 20-30 帧）
5. 采集时注意覆盖以下变化：
   - 棋盘格在画面中的**不同位置**（左、中、右、上、下）
   - 不同**距离**（远、中、近）
   - 不同**倾斜角度**（正面、左倾、右倾、前倾、后倾）
6. 按 **`q`** 键结束采集，脚本会自动计算内参并保存

输出文件：
```
~/Software/planb-robot/RobotCamCalib/outputs/intrinsics.yaml
```

### 3.3 确认 / 覆盖内参

如果不需要完整内参标定，可以直接从相机 SDK 读取出厂内参并手动写入：

1. 启动 rosci 环境并读取相机内参：
   ```bash
   cd ~/rosci_software
   source env_rosci_setup.bash
   # camera_d4x5f_000x.yaml 中的 x 取决于使用的相机
   # 例：头部相机对应 camera_d455f_0008.yaml
   python3 python/camera_example.py ~/rosci_software/config/planb-0001/camera_d4x5f_000x.yaml
   ```

2. 注意 terminal 输出中的以下值：
   ```
   fx, fy = rgb_focal_length
   cx, cy = rgb_principal_point
   ```

3. 打开内参文件并修改 K 矩阵：
   ```bash
   vim ~/Software/planb-robot/RobotCamCalib/outputs/intrinsics.yaml
   ```
   将 `K` 矩阵中的 `fx`, `fy`, `cx`, `cy` 替换为读取到的值。

### 3.4 内参输出格式

`intrinsics.yaml` 文件结构：

```yaml
image_size: [1280, 720]
K:
  - [642.29, 0.0, 634.26]      # [fx, 0, cx]
  - [0.0, 641.53, 366.30]      # [0, fy, cy]
  - [0.0, 0.0, 1.0]            # [0, 0, 1]
dist: [-0.0567, 0.0667, 2.6e-5, 7.7e-4, -0.0215]  # [k1, k2, p1, p2, k3]
fx: 642.29
fy: 641.53
cx: 634.26
cy: 366.30
rms: 0.0                       # 标定 RMS 重投影误差
mean_reproj_error: 0.0         # 平均重投影误差（像素）
```

---

## 4. 外参标定

### 4.1 准备 AprilTag

1. 打印 AprilTag 图案：
   ```
   文件路径：~/Software/planb-robot/RobotCamCalib/assets/extr_calib_apriltag.pdf
   ```
2. 打印规格：**A4 纸，边长至少大于10cm**
3. 将 AprilTag 剪下来，贴在平整的小板子上
4. 把板子贴在机器人的**手背**（即 `R_tcp` 关节对应的位置）

标定参数：
- Tag 家族：`tag36h11`
- Tag 尺寸：`0.1m`（100mm），以打印的AprilTag的二维码边长为主

### 4.2 让机器人进入拖拽模式

```bash
# 新开一个 terminal
cd ~/rosci_software
source env_rosci_setup.bash
python3 python/robot_task.py ~/rosci_software/config/planb-0001/robot_0001.yaml
# 输入 107 进入拖拽模式
```

### 4.3 配置外参标定脚本

打开 `extr_calib.py`，找到 `thirdview_realsense_xarm6_example()` 函数（腕部相机用 `wrist_realsense_xarm6_example()`），检查以下配置：

**1. URDF 路径**：确认指向正确的机器人 URDF 文件

**2. ROBOT_CONFIG**：确认指向 `robot_0001.yaml`

**3. ExtrinsicsCalibConfig 传参**：

```python
cfg = ExtrinsicsCalibConfig(
    robot_urdf_path=Path(URDF),
    cammount_link_name="L_D2",       # 相机安装的关节名称（URDF 中查找）
    tagmount_link_name="R_tcp",      # AprilTag 安装的关节名称
    K=K,                              # 内参矩阵（从 intrinsics.yaml 加载）
    output_file_path=current_dir / "outputs" / "extrinsics_rtest.yaml",
    tag_size=0.048,                   # AprilTag 物理尺寸（米）
    tag_family="tag36h11",
)
```

配置说明：

| 参数 | 说明 | 示例 |
|------|------|------|
| `cammount_link_name` | 相机安装在哪个 URDF link 上 | 头部相机 → `"L_D2"` |
| `tagmount_link_name` | AprilTag 贴在哪个 URDF link 上 | 右手手背 → `"R_tcp"` |
| `output_file_path` | 外参结果保存路径 | `outputs/extrinsics_rtest.yaml` |
| `tag_size` | AprilTag 黑色边框的物理边长 | `0.048`（48mm） |

### 4.4 运行外参标定

```bash
cd ~/Software/planb-robot/RobotCamCalib
python3 extr_calib.py
```

操作步骤：

1. 注意 terminal 中输出的端口号（默认 **8080**），在浏览器中打开：
   ```
   http://localhost:8080
   ```
   可以看到机器人的 URDF 3D 模型、相机画面和估计的坐标系

2. 按住机器人手臂上的拖拽按钮，移动手臂使 AprilTag 出现在相机视野内

3. 在 Web 界面右上角点击 **Append**
   - 注意！！！！保证机械臂静止的时候再 Append Observation，保证机械臂的关节角和当前的observation对齐！！！
   - 每次 Append 会记录当前的关节角度和 AprilTag 检测结果
   - 收集到 **≥ 8 个观测**后，算法会自动开始求解
   - Web 中会实时更新相机和 Tag 的估计位置

4. **重复拖拽**，保证：
   - 每次 pose 尽量不同（关节角度变化大）
   - AprilTag 始终在相机视野内且清晰可见
   - 覆盖不同的距离和角度组合
  

5. 持续到 Web 中相机位姿**收敛**（不再大幅变化），通常需要 **50-100 个观测**

6. 点击 **Save**，保存外参标定结果

### 4.5 外参输出格式

`extrinsics.yaml` 文件结构：

```yaml
X_CammountCam:                  # 相机安装关节 → 相机光心（4x4 齐次变换矩阵）
  - [-0.4158, -0.1833, -0.8908, 0.9190]    # [R | t]
  - [0.9091, -0.1102, -0.4017, 0.3287]     # 前3列: 旋转矩阵 R
  - [-0.0245, -0.9769, 0.2125, 0.2018]     # 最后1列: 平移向量 t (米)
  - [0, 0, 0, 1]

X_TagmountTag:                  # Tag安装关节 → AprilTag（4x4 齐次变换矩阵）
  - [0.0073, 0.0520, -0.9986, 0.0234]
  - [0.0250, -0.9983, -0.0518, -0.0017]
  - [-0.9997, -0.0246, -0.0085, 0.0588]
  - [0, 0, 0, 1]
```

---

## 5. 标定结果验证

标定完成后，可通过以下方式验证结果质量：

### 查看标定残差

运行标定时 terminal 会输出求解信息：

```
rot_err_deg_mean:  旋转残差均值（度）  — 建议 < 1.0°
rot_err_deg_max:   旋转残差最大值（度）— 建议 < 3.0°
trans_err_mean:    平移残差均值（米）  — 建议 < 0.005m
trans_err_max:     平移残差最大值（米）— 建议 < 0.01m
```

### 视觉验证

在 Viser Web 界面中：
- 估计的相机位置应与实际安装位置大致吻合
- 估计的 Tag 位置应在机器人手背附近
- 坐标系朝向应符合物理直觉

### 重投影验证（内参）

内参标定时关注：
- `rms` < 0.5 为优秀，< 1.0 为可接受
- `mean_reproj_error` < 0.3 像素为优秀

---

## 6. 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| AprilTag 检测不到 | Tag 不在视野内 / 太远 / 模糊 | 调整手臂位置，确保 Tag 清晰完整出现在画面中 |
| 外参不收敛 | 观测不够多样 | 增加观测数量，确保关节角度变化大且覆盖多种 pose |
| 标定结果偏差大 | 内参不准 / Tag 弯曲 | 确认内参正确；确保 Tag 贴在平整硬板上 |
| 相机被占用 | 其他进程占用相机 | 脚本会自动停止已有的 camera publisher，如仍有冲突手动 kill 相关进程 |
| Web 界面打不开 | 端口被占用 | 检查 terminal 输出的实际端口号，可能不是 8080 |
| 内参标定帧太少 | 棋盘格检测失败 | 确保光线充足、棋盘格完整在画面内、无运动模糊 |

---

## 附录 A: 关键参数速查

### ExtrinsicsCalibConfig

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_urdf_path` | Path | 必填 | 机器人 URDF 路径 |
| `cammount_link_name` | str | 必填 | 相机安装的 link 名称 |
| `tagmount_link_name` | str | 必填 | Tag 安装的 link 名称 |
| `K` | np.ndarray (3,3) | 必填 | 相机内参矩阵 |
| `output_file_path` | Path | `outputs/extrinsics_r.yaml` | 输出路径 |
| `tag_size` | float | 0.048 | AprilTag 边长（米） |
| `tag_family` | str | `tag36h11` | AprilTag 家族 |
| `ransac_sample_size` | int | 8 | 最少观测数 |

### 优化器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_iters` | 200 | Gauss-Newton 最大迭代次数 |
| `huber_delta_rot_deg` | 3.0 | Huber 旋转阈值（度） |
| `huber_delta_trans` | 0.01 | Huber 平移阈值（米） |
| `damping` | 1e-6 | Levenberg 阻尼因子 |

### 内参标定参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CHECKERBOARD` | (8, 6) | 棋盘格内角点数 |
| `SQUARE_SIZE` | 25.0 mm | 棋盘格方格边长 |
| `MIN_SAMPLES` | 12 | 最少有效检测帧数 |

---

## 附录 B: 文件目录结构

```
RobotCamCalib/
├── extr_calib.py                    # 外参标定主脚本
├── intr_calib.py                    # 内参标定脚本
├── 0_intr_calib.py                  # 内参标定（备选）
├── cameras.py                       # 相机接口封装
├── recorder_av_cam.py               # PyAV 录制工具
├── assets/
│   ├── intr_calib_checkerboard.pdf  # 内参标定用棋盘格（A4 打印）
│   ├── extr_calib_apriltag.pdf      # 外参标定用 AprilTag（A4 打印）
│   └── robots/xarm6/               # xArm6 URDF 参考
├── outputs/
│   ├── intrinsics.yaml              # 内参输出
│   ├── extrinsics.yaml              # 外参输出（头部相机）
│   ├── extrinsics_wrist.yaml        # 外参输出（腕部相机）
│   └── extrinsics_rtest.yaml        # 外参输出（测试）
└── PLANB_urdf_tcp_260212/           # PLANB 机器人 URDF
    ├── urdf/
    ├── meshes/
    └── config/
```

### PLANB 机器人关节映射

| 关节组 | 关节名称 | 说明 |
|--------|---------|------|
| 右臂 | JB1 - JB7 | 7 自由度 |
| 左臂 | JC1 - JC7 | 7 自由度 |
| 头部 | JD1 (yaw), JD2 (pitch) | 头部偏航/俯仰 |
| 相机安装点 | L_D2 | 头部相机 link |
| 右手 TCP | R_tcp | 右手末端（贴 AprilTag） |
