"""
TaskApp - 应用层
负责用户交互和流程控制
"""

import sys
import time
from TaskManager import TaskManager
from vision_to_rob import load_workpiece_coords
from vision_trigger import trigger_vision_system
from RobotCtrl import RobotCtrl

def main():

    # 机器人IP地址
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.10"
    # 模块圆心坐标文件地址
    folder_path = r"E:\圆心\1"
 
    # 创建 RobotCtrl 实例
    robot_ctrl = RobotCtrl(robot_ip)
    
    # # 复用 RobotCtrl 创建 TaskManager
    manager = TaskManager(robot_ctrl=robot_ctrl)

    print(f"\n===============================")
    print(f"=== UR5 RTDE 机器人控制程序 ===")
    print(f"===============================")
    print(f"目标机器人IP: {robot_ip}")
    print("\n⚠️  警告: 此程序包含机器人移动操作！")
    print("请确保：")
    print("  1. 机器人处于安全位置")
    print("  2. 工作区域无障碍物")
    print("  3. 已准备好急停装置")

    # 安全确认
    confirm = input("\n确认继续执行？(y/N): ")
    if confirm.lower() != 'y':
        print("用户取消操作")
        return

    # 连接机器人
    if not manager.connect():
        print("\n无法连接到机器人，请检查：")
        print("1. 机器人IP地址是否正确")
        print("2. 机器人是否已启动并处于远程控制模式")
        print("3. 网络连接是否正常")
        return

    print("\n======= 0. 触发视觉系统采图，用于摆正位姿 =======")
    # 复用已连接的 robot_ctrl
    success = trigger_vision_system(robot_ctrl)
    time.sleep(3)
    if not success:
        print("触发视觉系统采图失败,重新启动程序")
        return

    # 第一次加载坐标（用于摆正位姿）
    delta_xyz_mm, delta_rpy_deg = load_workpiece_coords(folder_path)
    #执行循环，一直到数据加载成功
    while delta_xyz_mm is None and delta_rpy_deg is None:
        success = trigger_vision_system(robot_ctrl)
        time.sleep(3)
        if not success:
            print("触发视觉系统采图失败,重新启动程序")
            return
        delta_xyz_mm, delta_rpy_deg = load_workpiece_coords(folder_path)
    
    # 摆正位姿：仅调整旋转角度，位置保持不变（设为0）
    pose_only_coords = [0, 0, 0] + delta_rpy_deg
    print(f"\n摆正位姿的工件坐标列表: {pose_only_coords}")
    print(f"  位置 (mm): X={pose_only_coords[0]:.3f}, Y={pose_only_coords[1]:.3f}, Z={pose_only_coords[2]:.3f}")
    print(f"  旋转 (deg): RX={pose_only_coords[3]:.3f}°, RY={pose_only_coords[4]:.3f}°, RZ={pose_only_coords[5]:.3f}°")


    try:
      
        # 执行任务流程
        print("\n======= 1. 设置移动速度 =======")
        manager.set_speed(0.08)
        manager.set_acceleration(0.5)

        print("\n======= 2. 获取当前TCP姿态 =======")
        manager.get_pose_base()

        print("\n======= 3. 移动到工件位置（摆正位姿） =======")
        if pose_only_coords:
            manager.move_to_workpiece(pose_only_coords)
            manager.wait_for_motion_complete(timeout=30)
         #   manager.move_tool_relative(dz_mm=70)
        else:
            print("跳过移动：工件坐标获取失败")
        
        print("\n======= 4. 触发视觉系统采图 =======")
        # 调用视觉触发函数（自动获取机器人数据并发送）
        success = trigger_vision_system(robot_ctrl)
        time.sleep(3)
        if not success:
            print("触发视觉系统采图失败,重新启动程序")
            return
        
        # 从文件加载坐标
        print("\n======= 5. 加载工件坐标 =======")

        delta_xyz_mm, delta_rpy_deg = load_workpiece_coords(folder_path)
        #执行循环，一直到数据加载成功
        while delta_xyz_mm is None and delta_rpy_deg is None:
            success = trigger_vision_system(robot_ctrl)
            time.sleep(3)
            if not success:
                print("触发视觉系统采图失败,重新启动程序")
                return
            delta_xyz_mm, delta_rpy_deg = load_workpiece_coords(folder_path)

        print("\n======= 6. 移动到工件位置（保持工件和相机相对平行的条件下） =======")  
        
        full_coords = delta_xyz_mm + delta_rpy_deg
        print(f"\n最终工件坐标列表: {full_coords}")
        print(f"  位置 (mm): X={full_coords[0]:.3f}, Y={full_coords[1]:.3f}, Z={full_coords[2]:.3f}")
        print(f"  旋转 (deg): RX={full_coords[3]:.3f}°, RY={full_coords[4]:.3f}°, RZ={full_coords[5]:.3f}°")
            
        manager.move_to_workpiece(full_coords)
        manager.wait_for_motion_complete(timeout=30)
  
        print("\n======= 7. 获取移动后的TCP姿态 =======")
        manager.get_pose_base()

        print("\n======= 8. 插入螺丝孔 =======")
        manager.move_tool_relative(dz_mm=15)

        print("\n======= 9. 是否返回采样位置 =======")
        #用户操作是否需要返回初始位置
        confirm = input("\n确认返回初始采样位置？(y/N): ")
        if confirm.lower() != 'y':
            print("用户取消操作")
            return
        success = manager.back_to_home()
        if not success:
            print("返回初始位置失败,重新启动程序")
            return

        print("\n======= 操作完成 =======")

    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        manager.disconnect()

if __name__ == "__main__":
    main()