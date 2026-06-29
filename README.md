# UR5 机器人视觉定位系统

基于 RTDE 协议的 UR5 机器人视觉定位控制系统，支持视觉触发、坐标转换和机器人运动控制。

---

## 目录结构

```
ur5_project/
├── RobotCtrl.py          # 控制层：RTDE通信接口
├── TaskManager.py        # 业务层：机器人控制逻辑
├── TaskApp.py            # 应用层：主程序入口
├── vision_trigger.py     # 视觉系统触发模块
├── vision_to_robot.py    # 视觉坐标转换模块
├── test.py               # 测试脚本
├── robot_api_doc.md      # API 文档
├── .gitignore            # Git 忽略配置
└── README.md             # 项目说明
```

---

## 模块说明

### 1. 控制层 - RobotCtrl

负责与 UR 机器人的底层 RTDE 通信：
- 建立/断开连接
- 获取 TCP 位姿（m, rad）
- 获取关节位置（rad）
- 执行线性移动

### 2. 业务层 - TaskManager

封装机器人业务逻辑和单位转换：
- 速度/加速度设置
- 基坐标系绝对移动（mm, deg）
- 工具坐标系相对移动
- 等待运动完成

### 3. 视觉触发 - vision_trigger

通过 TCP 客户端触发视觉系统：
- TCP Client 通信（ASCII 编码）
- 命令格式：`p,1,0,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz\r`
- 状态码解析（1-7）

### 4. 坐标转换 - vision_to_robot

视觉坐标到机器人坐标的转换：
- 手眼标定外参应用
- 四元数与旋转矩阵转换
- 工件坐标系偏移计算

---

## 安装依赖

```bash
# Python 3.8+
pip install numpy
pip install transforms3d
pip install scipy
pip install ur-rtde
```

---

## 使用方法

### 运行主程序

```bash
python TaskApp.py [机器人IP]

# 默认 IP: 192.168.0.10
python TaskApp.py

# 指定 IP
python TaskApp.py 192.168.1.100
```

### 测试视觉触发

```bash
python vision_trigger.py
```

### 测试机器人状态获取

```bash
python test.py
```

---

## 工作流程

```
1. 连接机器人
2. 触发视觉系统（获取工件坐标）
3. 移动到工件位置（摆正位姿）
4. 再次触发视觉系统（精确定位）
5. 移动到目标位置（完整6DOF）
6. 断开连接
```

---

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 机器人 IP | 192.168.0.10 | UR5 机器人地址 |
| 视觉服务器 | 127.0.0.1:50000 | 视觉系统地址 |
| 默认速度 | 0.25 m/s | 机器人移动速度 |
| 默认加速度 | 1.2 m/s² | 机器人加速度 |
| 数据目录 | E:\圆心\1 | JSON 文件路径 |

---

## 协议说明

### 视觉触发协议

- **协议类型**: TCP Client
- **编码方式**: ASCII
- **命令格式**: `p,1,0,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz\r`
- **终止符**: `\r`

### 状态码定义

| 状态码 | 含义 |
|--------|------|
| 1 | 成功 |
| 2 | 非法指令 |
| 3 | 工程未加载 |
| 4 | 无点云 |
| 5 | 无结果 |
| 6 | 规划失败 |
| 7 | 其他错误 |

---

## 注意事项

1. 确保机器人处于远程控制模式
2. 工作区域无障碍物
3. 准备好急停装置
4. 网络连接正常

---

## 许可证

MIT License