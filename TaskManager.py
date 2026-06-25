"""
TaskManager - 业务层
负责机器人业务逻辑和单位转换
"""

import math
from RobotCtrl import RobotCtrl

class TaskManager:
    def __init__(self, robot_ip=None, robot_ctrl=None):
        """
        初始化任务管理器
        
        参数:
            robot_ip: 机器人IP地址（创建新的 RobotCtrl）
            robot_ctrl: 已有的 RobotCtrl 实例（复用）
        """
        if robot_ctrl is not None:
            self.robot = robot_ctrl
        elif robot_ip is not None:
            self.robot = RobotCtrl(robot_ip)
        else:
            raise ValueError("必须提供 robot_ip 或 robot_ctrl")
        
        # 运动参数设置
        self.default_speed = 0.25        # m/s
        self.default_acceleration = 1.2  # m/s²
        self.speed_min = 0.01
        self.speed_max = 1.0
        self.acceleration_min = 0.1
        self.acceleration_max = 3.0

    def connect(self):
        """连接到机器人"""
        return self.robot.connect()

    def disconnect(self):
        """断开连接"""
        self.robot.disconnect()

    @property
    def connected(self):
        """检查连接状态"""
        return self.robot.connected

    def set_speed(self, speed):
        """设置默认速度（m/s）"""
        if self.speed_min <= speed <= self.speed_max:
            self.default_speed = speed
            print(f"速度已设置为: {speed:.3f} m/s")
            return True
        print(f"错误: 速度 {speed} 超出范围 [{self.speed_min}, {self.speed_max}] m/s")
        return False

    def set_acceleration(self, acceleration):
        """设置默认加速度（m/s²）"""
        if self.acceleration_min <= acceleration <= self.acceleration_max:
            self.default_acceleration = acceleration
            print(f"加速度已设置为: {acceleration:.3f} m/s²")
            return True
        print(f"错误: 加速度 {acceleration} 超出范围 [{self.acceleration_min}, {self.acceleration_max}] m/s²")
        return False

    def get_speed_settings(self):
        """查看当前速度设置"""
        print(f"当前速度设置:")
        print(f"  默认速度: {self.default_speed:.3f} m/s")
        print(f"  默认加速度: {self.default_acceleration:.3f} m/s²")
        print(f"  速度范围: [{self.speed_min}, {self.speed_max}] m/s")
        print(f"  加速度范围: [{self.acceleration_min}, {self.acceleration_max}] m/s²")

    def get_pose_base(self):
        """获取基坐标系下的TCP姿态"""
        pose = self.robot.get_tcp_pose()
        if pose is None:
            print("错误: 未连接到机器人")
            return None

        print(f"移动前坐标: {pose}")

        x, y, z, rx, ry, rz = pose
        x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000

        print(f"\n基坐标系下的当前姿态:")
        print(f"  位置 (m): x={x:.6f}, y={y:.6f}, z={z:.6f}")
        print(f"  位置 (mm): x={x_mm:.3f}, y={y_mm:.3f}, z={z_mm:.3f}")
        print(f"  旋转轴角 (rad): rx={rx:.6f}, ry={ry:.6f}, rz={rz:.6f}")
        print(f"  旋转轴角 (deg): rx={math.degrees(rx):.3f}°, ry={math.degrees(ry):.3f}°, rz={math.degrees(rz):.3f}°")

        return pose

    def get_joint_positions(self):
        """获取关节位置"""
        joints = self.robot.get_joint_positions()
        if joints is None:
            print("错误: 未连接到机器人")
            return None

        print(f"当前关节角度 (rad): {[round(j, 4) for j in joints]}")
        print(f"当前关节角度 (deg): {[round(math.degrees(j), 2) for j in joints]}")
        return joints

    def move_to_absolute_position(self, x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, speed=None, acceleration=None):
        """
        移动到绝对位置（基坐标系）
        参数: x_mm, y_mm, z_mm (mm), rx_deg, ry_deg, rz_deg (deg)
        """
        current_speed = speed if speed is not None else self.default_speed
        current_accel = acceleration if acceleration is not None else self.default_acceleration

        if not self._validate_speed(current_speed) or not self._validate_acceleration(current_accel):
            return False

        # 单位转换
        x, y, z = x_mm / 1000, y_mm / 1000, z_mm / 1000
        rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)

        print(f"\n移动到绝对位置（基坐标系）:")
        print(f"  目标位置 (mm): x={x_mm:.3f}, y={y_mm:.3f}, z={z_mm:.3f}")
        print(f"  目标旋转 (deg): rx={rx_deg:.3f}°, ry={ry_deg:.3f}°, rz={rz_deg:.3f}°")
        print(f"  速度: {current_speed:.3f} m/s, 加速度: {current_accel:.3f} m/s²")

        return self.robot.move_linear([x, y, z, rx, ry, rz], current_speed, current_accel)

    def move_tool_relative(self, dx_mm=0, dy_mm=0, dz_mm=0, drx_deg=0, dry_deg=0, drz_deg=0, speed=None, acceleration=None):
        """
        工具坐标系下相对移动
        参数: dx_mm, dy_mm, dz_mm (mm), drx_deg, dry_deg, drz_deg (deg)
        """
        current_speed = speed if speed is not None else self.default_speed
        current_accel = acceleration if acceleration is not None else self.default_acceleration

        if not self._validate_speed(current_speed) or not self._validate_acceleration(current_accel):
            return False

        # 单位转换
        dx, dy, dz = dx_mm / 1000, dy_mm / 1000, dz_mm / 1000
        drx, dry, drz = math.radians(drx_deg), math.radians(dry_deg), math.radians(drz_deg)

        # 获取当前TCP姿态（基坐标系）
        current_pose = self.robot.get_tcp_pose()
        if current_pose is None:
            return False

        # 从轴角提取当前旋转矩阵
        R = self._axis_angle_to_rotation_matrix(current_pose[3], current_pose[4], current_pose[5])
        
        # 将工具坐标系下的位移转换到基坐标系
        delta_x = R[0][0] * dx + R[0][1] * dy + R[0][2] * dz
        delta_y = R[1][0] * dx + R[1][1] * dy + R[1][2] * dz
        delta_z = R[2][0] * dx + R[2][1] * dy + R[2][2] * dz

        # 计算新位置
        target_x = current_pose[0] + delta_x
        target_y = current_pose[1] + delta_y
        target_z = current_pose[2] + delta_z

        # 计算新旋转（轴角组合）
        target_rx, target_ry, target_rz = self._compose_axis_angles(
            current_pose[3], current_pose[4], current_pose[5],
            drx, dry, drz
        )

        target_pose = [target_x, target_y, target_z, target_rx, target_ry, target_rz]

        print(f"\n工具坐标系下相对移动:")
        print(f"  位移 (mm): dx={dx_mm:.3f}, dy={dy_mm:.3f}, dz={dz_mm:.3f}")
        print(f"  旋转 (deg): drx={drx_deg:.3f}°, dry={dry_deg:.3f}°, drz={drz_deg:.3f}°")
        print(f"  速度: {current_speed:.3f} m/s, 加速度: {current_accel:.3f} m/s²")

        return self.robot.move_linear(target_pose, current_speed, current_accel)

    def move_to_workpiece(self, workpiece_coords, speed=None, acceleration=None):
        """
        移动到工件位置（相对工具坐标系）
        参数: workpiece_coords - 列表 [x, y, z, rx, ry, rz]，单位 mm 和 deg
        """
        if len(workpiece_coords) != 6:
            print("错误: 工件坐标必须包含6个元素 [x, y, z, rx, ry, rz]")
            return False
        
        x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = workpiece_coords
        
        print(f"移动到工件位置（相对工具坐标系）:")
        print(f"  工件坐标: [{x_mm:.3f}, {y_mm:.3f}, {z_mm:.3f}, {rx_deg:.3f}°, {ry_deg:.3f}°, {rz_deg:.3f}°]")
        
        return self.move_tool_relative(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, speed, acceleration)
    
    def _axis_angle_to_rotation_matrix(self, rx, ry, rz):
        """将轴角转换为旋转矩阵"""
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-10:
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        
        ax, ay, az = rx / angle, ry / angle, rz / angle
        c, s = math.cos(angle), math.sin(angle)
        cc = 1 - c
        
        return [
            [cc * ax * ax + c,     cc * ax * ay - az * s, cc * ax * az + ay * s],
            [cc * ay * ax + az * s, cc * ay * ay + c,     cc * ay * az - ax * s],
            [cc * az * ax - ay * s, cc * az * ay + ax * s, cc * az * az + c]
        ]
    
    def _rotation_matrix_to_axis_angle(self, R):
        """将旋转矩阵转换为轴角"""
        trace = R[0][0] + R[1][1] + R[2][2]
        if trace > 3 - 1e-10:
            return 0, 0, 0
        
        angle = math.acos((trace - 1) / 2)
        if angle < 1e-10:
            return 0, 0, 0
        
        denom = 2 * math.sin(angle)
        rx = (R[2][1] - R[1][2]) / denom
        ry = (R[0][2] - R[2][0]) / denom
        rz = (R[1][0] - R[0][1]) / denom
        
        return rx * angle, ry * angle, rz * angle
    
    def _compose_axis_angles(self, rx1, ry1, rz1, rx2, ry2, rz2):
        """组合两个轴角旋转（先应用 rx1,ry1,rz1，再应用 rx2,ry2,rz2）"""
        R1 = self._axis_angle_to_rotation_matrix(rx1, ry1, rz1)
        R2 = self._axis_angle_to_rotation_matrix(rx2, ry2, rz2)
        
        # 矩阵乘法：R = R1 * R2
        R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(3):
            for j in range(3):
                R[i][j] = sum(R1[i][k] * R2[k][j] for k in range(3))
        
        return self._rotation_matrix_to_axis_angle(R)

    def _validate_speed(self, speed):
        """验证速度范围"""
        if self.speed_min <= speed <= self.speed_max:
            return True
        print(f"错误: 速度 {speed} 超出范围 [{self.speed_min}, {self.speed_max}] m/s")
        return False

    def _validate_acceleration(self, acceleration):
        """验证加速度范围"""
        if self.acceleration_min <= acceleration <= self.acceleration_max:
            return True
        print(f"错误: 加速度 {acceleration} 超出范围 [{self.acceleration_min}, {self.acceleration_max}] m/s²")
        return False

    def wait_for_motion_complete(self, timeout=30, threshold=0.001):
        """
        等待机器人移动完成
        参数:
            timeout: 最大等待时间（秒）
            threshold: 速度阈值，低于此值认为已停止（m/s）
        """
        import time
        start_time = time.time()
        while time.time() - start_time < timeout:
            # 获取当前TCP速度
            tcp_speed = self.robot.get_tcp_speed()
            if tcp_speed is not None:
                # 计算速度大小
                speed_magnitude = (tcp_speed[0]**2 + tcp_speed[1]**2 + tcp_speed[2]**2)**0.5
                if speed_magnitude < threshold:
                    print(f"✓ 移动完成（速度: {speed_magnitude:.6f} m/s）")
                    return True
            time.sleep(0.1)
        print(f"✗ 等待超时（{timeout}秒）")
        return False


    def back_to_home(self):
        """返回初始位置"""
        # 执行Z轴上移和绝对坐标移动
        if self.move_tool_relative(dz_mm=-50):
            print("✓ Z轴上移成功")
            print("等待移动完成...")
            self.wait_for_motion_complete(timeout=30)
            
            print("\n=== 移动到指定绝对坐标位置 ===")
            target_position = [-453.55, -198.15, 480.67, 173.30, -47.24, 0.34]
            if self.move_to_absolute_position(*target_position):
                print("✓ 移动到绝对坐标成功")
                return True
            else:
                print("✗ 移动到绝对坐标失败")  
                return False
        else:
            print("✗ Z轴上移失败，跳过后续操作")
            print("跳过回到初始位置")
            return False
