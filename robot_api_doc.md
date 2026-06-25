# 机器人控制模块 API 文档

## 概述

本文档描述了 UR5 机器人控制项目中的两个核心模块：
- **RobotCtrl.py** - 控制层，负责与 UR 机器人的底层 RTDE 通信
- **TaskManager.py** - 业务层，负责机器人业务逻辑和单位转换

---

# RobotCtrl.py - 机器人控制层

## 功能定位

`RobotCtrl` 是与 UR 机器人底层 RTDE（Real-Time Data Exchange）通信的控制类，负责建立连接、获取机器人状态和执行移动指令。

## 类结构

```python
class RobotCtrl:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip      # 机器人IP地址
        self.rtde_c = None            # RTDE控制接口
        self.rtde_r = None            # RTDE接收接口
        self.connected = False        # 连接状态
```

## 方法说明

### 连接管理

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `connect()` | 建立 RTDE 连接 | 无 | `True`（成功）/ `False`（失败） |
| `disconnect()` | 断开 RTDE 连接 | 无 | 无 |

### 状态获取

| 方法 | 功能 | 返回值 |
|------|------|--------|
| `get_tcp_pose()` | 获取 TCP 姿态 | `[x, y, z, rx, ry, rz]` 或 `None` |
| `get_joint_positions()` | 获取关节位置 | `[q1, q2, q3, q4, q5, q6]` 或 `None` |
| `get_tcp_speed()` | 获取 TCP 速度 | `[vx, vy, vz]` 或 `None` |

### 运动控制

| 方法 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `move_linear(target_pose, speed, acceleration)` | 执行线性移动 | 目标位姿、速度、加速度 | `True`/`False` |

## 单位说明

| 数据类型 | 位置单位 | 旋转单位 | 速度单位 |
|----------|----------|----------|----------|
| 原始数据 | 米 (m) | 弧度 (rad) | m/s |

## 使用示例

```python
from RobotCtrl import RobotCtrl

# 初始化机器人控制器
robot = RobotCtrl("192.168.1.100")

# 建立连接
if robot.connect():
    # 获取当前TCP姿态
    pose = robot.get_tcp_pose()
    print(f"当前TCP姿态: {pose}")
    
    # 获取关节位置
    joints = robot.get_joint_positions()
    print(f"当前关节位置: {joints}")
    
    # 执行线性移动（单位：m, rad）
    target_pose = [-0.45, 0.2, 0.3, 0, 1.57, 0]
    robot.move_linear(target_pose, speed=0.25, acceleration=1.2)
    
    # 断开连接
    robot.disconnect()
```

---

# TaskManager.py - 业务逻辑层

## 功能定位

`TaskManager` 是业务逻辑层，封装机器人控制逻辑，提供单位转换（米↔毫米、弧度↔角度）和高级运动接口。

## 类结构

```python
class TaskManager:
    def __init__(self, robot_ip):
        self.robot = RobotCtrl(robot_ip)  # 机器人控制器
        
        # 运动参数
        self.default_speed = 0.25        # 默认速度 (m/s)
        self.default_acceleration = 1.2  # 默认加速度 (m/s²)
        self.speed_min = 0.01
        self.speed_max = 1.0
        self.acceleration_min = 0.1
        self.acceleration_max = 3.0
```

## 方法分类

### 连接管理

| 方法 | 说明 |
|------|------|
| `connect()` | 连接到机器人 |
| `disconnect()` | 断开连接 |
| `connected` | 检查连接状态（属性） |

### 参数设置

| 方法 | 说明 |
|------|------|
| `set_speed(speed)` | 设置默认速度（m/s） |
| `set_acceleration(acceleration)` | 设置默认加速度（m/s²） |
| `get_speed_settings()` | 查看当前速度设置 |

### 状态获取

| 方法 | 说明 |
|------|------|
| `get_pose_base()` | 获取基坐标系下的 TCP 姿态（打印详细信息） |
| `get_joint_positions()` | 获取关节位置（打印详细信息） |

### 运动控制

| 方法 | 说明 | 参数单位 |
|------|------|----------|
| `move_to_absolute_position(x, y, z, rx, ry, rz)` | 移动到绝对位置（基坐标系） | mm, deg |
| `move_tool_relative(dx, dy, dz, drx, dry, drz)` | 工具坐标系下相对移动 | mm, deg |
| `move_to_workpiece(workpiece_coords)` | 移动到工件位置 | mm, deg |
| `wait_for_motion_complete(timeout, threshold)` | 等待移动完成 | 秒, m/s |

### 内部方法

| 方法 | 说明 |
|------|------|
| `_axis_angle_to_rotation_matrix(rx, ry, rz)` | 轴角转旋转矩阵 |
| `_rotation_matrix_to_axis_angle(R)` | 旋转矩阵转轴角 |
| `_compose_axis_angles(rx1, ry1, rz1, rx2, ry2, rz2)` | 组合两个轴角旋转 |
| `_validate_speed(speed)` | 验证速度范围 |
| `_validate_acceleration(acceleration)` | 验证加速度范围 |

## 坐标系统

```
基坐标系（Base Frame）:
    ↗ Y
    │
    └──→ X
   /
  Z (垂直向上)

工具坐标系（Tool Frame）:
    以 TCP 为原点，工具指向为 Z 轴
```

## 使用示例

```python
from TaskManager import TaskManager

# 初始化任务管理器
manager = TaskManager("192.168.1.100")

# 连接到机器人
if manager.connect():
    # 设置运动参数
    manager.set_speed(0.08)
    manager.set_acceleration(0.5)
    
    # 获取当前姿态
    manager.get_pose_base()
    
    # 移动到绝对位置（基坐标系）
    manager.move_to_absolute_position(
        x_mm=-450, y_mm=200, z_mm=350,
        rx_deg=0, ry_deg=90, rz_deg=0
    )
    
    # 等待移动完成
    manager.wait_for_motion_complete()
    
    # 工具坐标系下相对移动（Z轴上移50mm）
    manager.move_tool_relative(dz_mm=50)
    
    # 断开连接
    manager.disconnect()
```

---

## 模块关系

```
TaskApp.py (应用层)
    │
    └── TaskManager.py (业务层)
            │
            └── RobotCtrl.py (控制层)
                    │
                    └── RTDE库 (底层通信)
```

| 层级 | 职责 | 接口单位 |
|------|------|----------|
| 应用层 | 流程控制、用户交互 | - |
| 业务层 | 单位转换、高级接口 | mm, deg |
| 控制层 | RTDE通信、低级接口 | m, rad |

---

## 版本信息

- **创建日期**: 2026-06-22
- **适用机器人**: UR5
- **通信协议**: RTDE
- **编程语言**: Python 3.x