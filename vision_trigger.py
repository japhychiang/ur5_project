"""
视觉系统触发模块 - Vision Trigger Module
功能：通过TCP客户端发送触发指令，支持状态码解析
协议：TCP Client，ASCII编码
远程主机：127.0.0.1:50000

命令格式：
    p,1,0,6个关节角(deg),6个法兰位姿(mm,deg)\r
    示例: p,1,0,-90.000,-45.000,120.000,-60.000,90.000,0.000,-452.123,215.456,350.789,-1.232,0.567,89.175\r

状态码说明：
    1 - 成功
    2 - 非法指令
    3 - 工程未加载
    4 - 无点云
    5 - 无结果
    6 - 规划失败
    7 - 其他错误

使用方法：
    from vision_trigger import VisionTrigger, trigger_vision_system
    
    # 方式1：类方法触发
    trigger = VisionTrigger()
    if trigger.connect():
        result = trigger.trigger_with_robot(robot_ctrl)
        trigger.disconnect()
    
    # 方式2：使用独立函数（一行代码完成完整触发）
    trigger_vision_system(robot_ctrl)
"""

import socket
import logging
from typing import Optional, Dict, Any
from RobotCtrl import RobotCtrl


# ============================================================================
# 配置和常量
# ============================================================================
# 日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('VisionTrigger')

# 状态码定义
class VisionStatus:
    """视觉系统状态码"""
    SUCCESS = 1
    ILLEGAL_COMMAND = 2
    PROJECT_NOT_LOADED = 3
    NO_POINT_CLOUD = 4
    NO_RESULT = 5
    PLANNING_FAILED = 6
    OTHER_ERROR = 7

# 状态码映射表
VISION_STATUS_CODES = {
    VisionStatus.SUCCESS: {
        'code': VisionStatus.SUCCESS,
        'status': 'success',
        'message': '成功'
    },
    VisionStatus.ILLEGAL_COMMAND: {
        'code': VisionStatus.ILLEGAL_COMMAND,
        'status': 'illegal_command',
        'message': '非法指令'
    },
    VisionStatus.PROJECT_NOT_LOADED: {
        'code': VisionStatus.PROJECT_NOT_LOADED,
        'status': 'project_not_loaded',
        'message': '工程未加载'
    },
    VisionStatus.NO_POINT_CLOUD: {
        'code': VisionStatus.NO_POINT_CLOUD,
        'status': 'no_point_cloud',
        'message': '无点云'
    },
    VisionStatus.NO_RESULT: {
        'code': VisionStatus.NO_RESULT,
        'status': 'no_result',
        'message': '无结果'
    },
    VisionStatus.PLANNING_FAILED: {
        'code': VisionStatus.PLANNING_FAILED,
        'status': 'planning_failed',
        'message': '规划失败'
    },
    VisionStatus.OTHER_ERROR: {
        'code': VisionStatus.OTHER_ERROR,
        'status': 'other_error',
        'message': '其他错误'
    },
}


# ============================================================================
# 工具函数
# ============================================================================

def parse_response(response: Optional[str]) -> Optional[Dict[str, Any]]:
    """
    解析视觉系统返回的数据，匹配状态码
    
    参数:
        response: 视觉系统返回的原始字符串
        
    返回:
        dict: 包含状态码、状态和消息的字典，如果无法解析返回None
    """
    if response is None:
        return None
    
    response = response.strip()
    
    # 尝试提取第一个字符作为状态码
    for status_code in VISION_STATUS_CODES.keys():
        if response.startswith(str(status_code)):
            status_info = VISION_STATUS_CODES[status_code].copy()
            
            # 提取描述信息
            if ',' in response:
                parts = response.split(',', 1)
                status_info['raw_code'] = parts[0]
                status_info['description'] = parts[1].strip() if len(parts) > 1 else ''
            else:
                status_info['raw_code'] = str(status_code)
                status_info['description'] = ''
            
            return status_info
    
    # 无法解析状态码
    return {
        'code': -1,
        'status': 'unknown',
        'message': '未知状态',
        'raw_response': response
    }


# ============================================================================
# 视觉触发器类
# ============================================================================

class VisionTrigger:
    """
    视觉系统触发客户端
    
    用于向视觉系统发送触发指令并接收返回数据
    
    命令格式:
        p,1,0,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz\r
    
    示例:
        trigger = VisionTrigger()
        if trigger.connect():
            result = trigger.trigger_with_robot(robot_ctrl)
            trigger.disconnect()
    """
    
    def __init__(
        self,
        host: str = '127.0.0.1',
        port: int = 50000,
        timeout: float = 10.0,
        terminator: str = '\r'
    ):
        """
        初始化视觉触发客户端
        
        参数:
            host: 远程主机地址，默认 127.0.0.1
            port: 远程主机端口，默认 50000
            timeout: 连接超时时间（秒），默认 10
            terminator: 命令终止符，默认 '\r'
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.terminator = terminator
        self.socket: Optional[socket.socket] = None
        self.connected = False
        
        logger.info(f"VisionTrigger initialized: {self.host}:{self.port}")
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
        return False
    
    def connect(self) -> bool:
        """
        连接到视觉系统服务器
        
        返回:
            bool: 连接成功返回True，失败返回False
        """
        if self.connected:
            logger.warning("Already connected")
            return True
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            
            logger.info(f"Connecting to {self.host}:{self.port}...")
            self.socket.connect((self.host, self.port))
            self.connected = True
            logger.info("Connection established")
            return True
            
        except ConnectionRefusedError:
            logger.error(f"Connection refused: {self.host}:{self.port}")
            return False
        except socket.timeout:
            logger.error(f"Connection timeout: {self.timeout}s")
            return False
        except Exception as e:
            logger.error(f"Connection error: {e}")
            return False
    
    def send_command(self, command: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        发送命令到视觉系统
        
        参数:
            command: 要发送的命令，如果为None则使用默认命令
        
        返回:
            dict: 包含状态码和详细信息的字典，如果失败返回 None
        """
        if not self.connected or self.socket is None:
            logger.error("Not connected, call connect() first")
            return None
        
        command = command or self.command
        full_command = f"{command}{self.terminator}"
        
        try:
            # 发送命令
            logger.info(f"Sending command: {repr(full_command)}")
            self.socket.sendall(full_command.encode('ascii'))
            
            # 接收返回数据
            logger.info("Waiting for response...")
            response = b""
            
            while True:
                try:
                    self.socket.settimeout(5.0)
                    data = self.socket.recv(1024)
                    if not data:
                        break
                    response += data
                    if self.terminator.encode('ascii') in response:
                        break
                except socket.timeout:
                    logger.warning("Receive timeout")
                    break
            
            # 处理返回数据
            if response:
                response_str = response.decode('ascii').strip('\r\n')
                self._last_response = response_str  # 保存原始响应供调试
                logger.info(f"Received response: {repr(response_str)}")
                
                result = parse_response(response_str)
                return result
            else:
                logger.warning("No response received")
                return result
                
        except socket.timeout:
            logger.error("Receive timeout")
            return None
        except Exception as e:
            logger.error(f"Send command error: {e}")
            return None
    
    def trigger_with_robot_data(
        self,
        joint_positions_deg: list,
        tcp_pose_mm_deg: list
    ) -> Optional[Dict[str, Any]]:
        """
        触发视觉系统（带机器人数据）
        
        命令格式: p,1,0,6个关节角,6个法兰位姿,结尾以\r
        例如: p,1,0,10.5,20.3,30.1,40.2,50.5,60.3,100.5,200.3,300.2,1.5,2.3,3.1\r
        
        参数:
            joint_positions_deg: 6个关节角度（单位：度）
            tcp_pose_mm_deg: 6个法兰位姿 [x,y,z,rx,ry,rz]（单位：mm和度）
        
        返回:
            dict: 包含状态码和详细信息的字典，如果失败返回 None
        """
        if not self.connected:
            if not self.connect():
                return None
        
        # 构建命令
        # 格式: p,1,0,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz
        joint_str = ','.join([f"{j:.3f}" for j in joint_positions_deg])
        pose_str = ','.join([f"{p:.3f}" for p in tcp_pose_mm_deg])
        
        command = f"p,1,0,{joint_str},{pose_str}"
        
        logger.info(f"构建命令: {command}")
        
        return self.send_command(command)
    
    def trigger_with_robot(self, robot_ctrl) -> Optional[Dict[str, Any]]:
        """
        触发视觉系统（自动从机器人获取数据）
        
        此方法会自动从机器人控制器获取当前状态并发送给视觉系统
        
        参数:
            robot_ctrl: 机器人控制器对象（需支持 get_tcp_pose() 和 get_joint_positions() 方法）
        
        返回:
            dict: 包含状态码和详细信息的字典，如果失败返回 None
        """
        import math
        
        print("\n获取机器人当前状态...")
        
        # 获取TCP位姿（单位：m和rad）
        tcp_pose = robot_ctrl.get_tcp_pose()
        if tcp_pose is None:
            print("✗ 无法获取TCP位姿")
            return None
        
        # 转换单位：m -> mm, rad -> deg
        tcp_pose_mm_deg = [
            tcp_pose[0] * 1000,
            tcp_pose[1] * 1000,
            tcp_pose[2] * 1000,
            math.degrees(tcp_pose[3]),
            math.degrees(tcp_pose[4]),
            math.degrees(tcp_pose[5]),
        ]
        
        # 获取关节角度（单位：rad）
        joint_positions = robot_ctrl.get_joint_positions()
        if joint_positions is None:
            print("✗ 无法获取关节角度")
            return None
        
        # 转换单位：rad -> deg
        joint_positions_deg = [math.degrees(j) for j in joint_positions]
        
        # 打印获取的数据
        print(f"  TCP位姿 (mm, deg): [{tcp_pose_mm_deg[0]:.3f}, {tcp_pose_mm_deg[1]:.3f}, {tcp_pose_mm_deg[2]:.3f}, {tcp_pose_mm_deg[3]:.3f}°, {tcp_pose_mm_deg[4]:.3f}°, {tcp_pose_mm_deg[5]:.3f}°]")
        print(f"  关节角度 (deg): [{joint_positions_deg[0]:.3f}°, {joint_positions_deg[1]:.3f}°, {joint_positions_deg[2]:.3f}°, {joint_positions_deg[3]:.3f}°, {joint_positions_deg[4]:.3f}°, {joint_positions_deg[5]:.3f}°]")
        
        # 调用带数据的触发方法
        return self.trigger_with_robot_data(joint_positions_deg, tcp_pose_mm_deg)
    
    def disconnect(self):
        """
        断开与服务器的连接
        """
        if self.socket:
            try:
                self.socket.close()
                logger.info("Disconnected")
            except Exception as e:
                logger.warning(f"Disconnect error: {e}")
        self.socket = None
        self.connected = False
    
    def is_connected(self) -> bool:
        """
        检查是否已连接
        
        返回:
            bool: 连接状态
        """
        return self.connected


# ============================================================================
# 独立函数
# ============================================================================

def trigger_vision_system(robot_ctrl, host='127.0.0.1', port=50000):
    """
    触发视觉系统采图（独立函数）
    
    封装连接、获取机器人数据、发送触发命令、断开的完整流程
    
    参数:
        robot_ctrl: 机器人控制器对象（需支持 get_tcp_pose() 和 get_joint_positions() 方法）
        host: 视觉系统服务器地址，默认 127.0.0.1
        port: 视觉系统服务器端口，默认 50000
    
    返回:
        bool: 触发成功返回True，失败返回False
    """
    vision_trigger = VisionTrigger(host=host, port=port)

    if vision_trigger.connect():
        result = vision_trigger.trigger_with_robot(robot_ctrl)

        # # 调试输出
        # print(f"\n【调试信息】")
        # print(f"原始响应: {repr(vision_trigger._last_response) if hasattr(vision_trigger, '_last_response') else '未记录'}")
        # print(f"解析结果: {result}")

        vision_trigger.disconnect()
        
        if result and result['code'] == VisionStatus.SUCCESS:
            print("✓ 视觉系统触发成功")
            return True
        else:
            print(f"✗ 视觉系统触发失败: {result['message'] if result else '无响应'}")
            return False
    else:
        print("✗ 视觉系统连接失败")
        return False

if __name__ == "__main__":
    from RobotCtrl import RobotCtrl
    
    print("=" * 60)
    print("  视觉触发测试")
    print("=" * 60)
    
    # 机器人IP地址
    robot_ip = "192.168.0.10"
    
    # 创建机器人控制器
    print("\n[1] 创建机器人控制器...")
    robot_ctrl = RobotCtrl(robot_ip)
    
    # 连接到机器人
    print("\n[2] 连接到机器人...")
    if not robot_ctrl.connect():
        print("✗ 机器人连接失败，无法继续测试")
        print("请检查：")
        print("  1. 机器人IP地址是否正确")
        print("  2. 机器人是否已启动并处于远程控制模式")
        print("  3. 网络连接是否正常")
        exit(1)
    
    print("✓ 机器人连接成功")
    
    # 触发视觉系统
    print("\n[3] 触发视觉系统...")
    success = trigger_vision_system(robot_ctrl)
    
    # 断开机器人连接
    print("\n[4] 断开机器人连接...")
    robot_ctrl.disconnect()
    
    print("\n" + "=" * 60)
    if success:
        print("  测试完成：视觉触发成功")
    else:
        print("  测试完成：视觉触发失败")
    print("=" * 60)