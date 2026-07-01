"""
手动返回初始位置测试
"""

from TaskManager import TaskManager 

def Manual_BackToHome():
    # 创建任务管理器
    manager = TaskManager("192.168.0.10")
    
    # 连接机器人
    if not manager.connect():
        print("连接失败")
        return
    
    # 设置速度
    manager.set_speed(0.08)
    manager.set_acceleration(0.5)
    
    # 获取当前位置
    print("当前位置：")
    manager.get_pose_base()
    
    # 操作确认：是否回到初始位置
    return_home = input("\n是否需要回到初始位置？(y/N): ")

    if return_home.lower() == 'y':
        print("\n=== 回到初始位置 ===")
        print(f"当前连接状态: {'已连接' if manager.connected else '未连接'}")
        
        # 执行Z轴上移和绝对坐标移动
        if manager.move_tool_relative(dz_mm=-50):
            print("✓ Z轴上移成功")
            print("等待移动完成...")
            manager.wait_for_motion_complete(timeout=30)
            
            print("\n=== 移动到指定绝对坐标位置 ===")
            target_position = [-453.55, -198.15, 480.67, 173.30, -47.24, 0.34]
            if manager.move_to_absolute_position(*target_position):
                print("✓ 移动到绝对坐标成功")
            else:
                print("✗ 移动到绝对坐标失败")  
        else:
            print("✗ Z轴上移失败，跳过后续操作")

        print("跳过回到初始位置")

    print("\n=== 7. 获取最终TCP姿态 ===")
    manager.get_pose_base()

    print("\n=== 操作完成 ===")

    # 断开连接
    manager.disconnect()


if __name__ == "__main__":   
    Manual_BackToHome()
