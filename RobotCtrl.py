"""
RobotCtrl - 控制层
负责与UR机器人的底层RTDE通信
"""

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


class RobotCtrl:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.rtde_c = None
        self.rtde_r = None
        self.connected = False

    def connect(self):
        """建立RTDE连接"""
        try:
            self.rtde_c = RTDEControlInterface(self.robot_ip)
            self.rtde_r = RTDEReceiveInterface(self.robot_ip)
            self.connected = True
            print(f"✓ 成功连接到机器人: {self.robot_ip}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    def disconnect(self):
        """断开RTDE连接"""
        if self.rtde_c:
            self.rtde_c.stopScript()
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()
        self.connected = False
        print("已断开与机器人的连接")

    def get_tcp_pose(self):
        """获取TCP姿态（原始单位：m, rad）"""
        if not self.connected:
            return None
        return self.rtde_r.getActualTCPPose()

    def get_joint_positions(self):
        """获取关节位置（原始单位：rad）"""
        if not self.connected:
            return None
        return self.rtde_r.getActualQ()

    def get_tcp_speed(self):
        """获取TCP速度（原始单位：m/s）"""
        if not self.connected:
            return None
        return self.rtde_r.getActualTCPSpeed()

    def move_linear(self, target_pose, speed, acceleration):
        """
        执行线性移动
        target_pose: [x, y, z, rx, ry, rz]（单位：m, rad）
        """
        if not self.connected:
            return False
        try:
            self.rtde_c.moveL(target_pose, speed, acceleration)
            return True
        except Exception as e:
            print(f"移动失败: {e}")
            return False
