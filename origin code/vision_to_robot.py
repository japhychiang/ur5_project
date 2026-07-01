#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import os
import json
from scipy.spatial.transform import Rotation as R


def calculate_delta_T( object_ ):
    """
    基于 SciPy 库计算当前法兰需要移动的相对变换偏差 delta_T
    :param object_：法兰坐标系下目标物体的位姿矩阵，包含位置和姿态信息，格式为[tx, ty, tz, qw, qx, qy, qz]。
    输出参数：
    delta_xyz_mm：相机到法兰的相对位置，单位为毫米。
    delta_wxyz：相机到法兰的相对姿态，单位为度。
    """
    # 1. 获取当前相机到物体的位姿变换矩阵 T_co
    T_co, _, _ = get_transform_matrix(object_)

    # 2. 定义期望的目标位姿 T_fo_desired (此处设为单位阵作为示例)
    target_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    T_fo_desired, _, _ = get_transform_matrix(target_pose)

    # 3. 核心偏差矩阵计算：delta_T = T_cf @ T_co @ inv(T_fo_desired)
    delta_T = T_co @ np.linalg.inv(T_fo_desired)

    # 4. 解耦偏差矩阵获取平移和旋转部分
    t = delta_T[0:3, 3]
    r_matrix = delta_T[0:3, 0:3]

    # 5. 使用 SciPy 将偏差旋转矩阵转为四元数，并重构为 [qw, qx, qy, qz] 格式
    rot_delta = R.from_matrix(r_matrix)
    qx, qy, qz, qw = rot_delta.as_quat() # SciPy 返回格式为 [qx, qy, qz, qw]
    delta_wxyz = [qw, qx, qy, qz]

    # 6. 使用 SciPy 的 as_rotvec 提取旋转向量 (等价于原 transforms3d 的 axis * angle)
    rot_vector = rot_delta.as_rotvec()  # 弧度制
    rx_deg, ry_deg, rz_deg = np.degrees(rot_vector) # 转换为度

    # 7. 位移转换为毫米
    delta_xyz_mm = t * 1000

    # 8. 格式化控制台输出
    print("ΔT (已简化外部 T_cf 输入):")
    print(f"  位置 (xyz_mm): [{delta_xyz_mm[0]:.6f}, {delta_xyz_mm[1]:.6f}, {delta_xyz_mm[2]:.6f}]")
    print(f"  姿态 (wxyz): [{delta_wxyz[0]:.6f}, {delta_wxyz[1]:.6f}, {delta_wxyz[2]:.6f}, {delta_wxyz[3]:.6f}]")
    print(f"  旋转角 (deg): [rx={rx_deg:.3f}°, ry={ry_deg:.3f}°, rz={rz_deg:.3f}°]")

    return delta_xyz_mm.tolist(), [rx_deg, ry_deg, rz_deg]  

def get_latest_json_file(folder_path):
    json_files = []
    for f in os.listdir(folder_path):
        if f.endswith('.json') and not f.startswith('converted'):
            parts = f.split('_')
            if len(parts) >= 2:
                num_part = parts[1].split('.')[0]
                if num_part.isdigit():
                    json_files.append(f)
    
    if not json_files:
        raise ValueError("No valid JSON files found in the directory")
    
    latest_file = max(json_files, key=lambda x: int(x.split('_')[1].split('.')[0]))
    return os.path.join(folder_path, latest_file)


def get_static_camera_to_flange_pose():
    """
    获取固化的相机到法兰的外参位姿数据（手眼标定结果）
    :return: [x, y, z, qw, qx, qy, qz] (单位：m, rad)
    """
    # 固定的外参数据，分别为平移(m)和四元数(w, x, y, z)
    return [0.118718, 0.040764, 0.085525, 0.707883, -0.010377, -0.009907, 0.706184]

def get_transform_matrix(pose_data):
    """
    根据给定的位姿数据 [x, y, z, qw, qx, qy, qz] (单位：m, rad)
    构建 4x4 齐次变换矩阵
    """
    # 1. 提取平移向量
    t = np.array(pose_data[0:3]) # [x_t, y_t, z_t]
    
    # 2. 提取四元数 [qw, qx, qy, qz] 
    qw, qx, qy, qz = pose_data[3], pose_data[4], pose_data[5], pose_data[6]
    
    # Scipy 接受的四元数顺序为 [qx, qy, qz, qw]，将四元数转换为旋转矩阵
    rot = R.from_quat([qx, qy, qz, qw])
    
    # 3. 获得 3x3 旋转矩阵
    R_matrix = rot.as_matrix()
    
    # 4. 构建 4x4 齐次变换矩阵 T
    T = np.eye(4)
    T[0:3, 0:3] = R_matrix
    T[0:3, 3] = t
    
    return T, R_matrix, t

def transform_pose(A_pose, T_CF):
    """
    将相机坐标系下的 7维位姿向量 A 转换到法兰坐标系下的 7维位姿向量 B
    
    :param A_pose: [x, y, z, qw, qx, qy, qz] 
                   注意：输入的位置单位为 m，姿态四元数为 (w, x, y, z) 格式
    :param T_CF: 相机到法兰的 4x4 齐次变换矩阵 (内部位置单位采用米 m)
    
    :return: B_pose: [x, y, z, qw, qx, qy, qz] (单位为 m 和 rad)
    """
    # ==================== 1. 位置部分的转换 ====================
    # 提取 A 的位置
    A_pos_m = np.array(A_pose[0:3])
    A_pos_homo = np.append(A_pos_m, 1.0) # 齐次化
    
    # 齐次矩阵乘法: B_pos = T_CF * A_pos
    B_pos_m = np.dot(T_CF, A_pos_homo)[0:3]
    
    # ==================== 2. 姿态部分的转换 ====================
    # 提取 A 的四元数 (输入顺序为 w, x, y, z)
    qw_a, qx_a, qy_a, qz_a = A_pose[3], A_pose[4], A_pose[5], A_pose[6]
    
    # Scipy 内部四元数格式为 [x, y, z, w]
    R_A = R.from_quat([qx_a, qy_a, qz_a, qw_a])
    
    # 提取外参的旋转矩阵 (从 T_CF 的左上角 3x3 截取)
    R_CF = T_CF[0:3, 0:3]
    R_CF_obj = R.from_matrix(R_CF)
    
    # 姿态乘法原理 (基于旋转矩阵乘法)：
    # 新的旋转 = 外参旋转 * 原始旋转  ==> R_B = R_CF * R_A
    R_B = R_CF_obj * R_A
    
    # 将结果转换回四元数，格式依然原生为 [qx, qy, qz, qw]
    B_quat_scipy = R_B.as_quat()
    # 重新整理成用户习惯的 (w, x, y, z) 格式
    qx_b, qy_b, qz_b, qw_b = B_quat_scipy[0], B_quat_scipy[1], B_quat_scipy[2], B_quat_scipy[3]
    
    # 组合为 7维输出位姿向量 B
    B_pose = [
        B_pos_m[0], B_pos_m[1], B_pos_m[2], # X, Y, Z (m)
        qw_b, qx_b, qy_b, qz_b              # qw, qx, qy, qz
    ]
    
    return B_pose

def offset_along_workpiece_z(B_pose, offset_distance):
    """
    将位姿 B 沿着工件坐标系（即 B 点自身姿态）的 Z 轴移动指定距离 a
    
    数学原理：
    1. 提取 B 点相对于法兰的旋转矩阵 R_B
    2. 工件坐标系下的 Z 轴方向单位向量为 [0, 0, 1]^T
    3. 将其转换到法兰系中的方向向量： direction = R_B * [0, 0, 1]^T (即 R_B 的第三列)
    4. 沿该方向平移 offset_distance： C_pos = B_pos + direction * a
    """
    C_pose = list(B_pose)
    qw, qx, qy, qz = B_pose[3], B_pose[4], B_pose[5], B_pose[6]
    
    # 1. 构建旋转对象并提取 3x3 旋转矩阵 R_B
    r = R.from_quat([qx, qy, qz, qw])
    R_B = r.as_matrix()
    
    # 2. 获取工件坐标系 Z 轴在法兰系下的映射（即旋转矩阵的第三列：R_B[:, 2]）
    workpiece_z_in_flange = R_B[:, 2]
    
    # 3. 进行平移计算,C=B点坐标+法兰系下解析出来的偏移量，最终计算出来C
    C_pose[0:3] = np.array(B_pose[0:3]) + workpiece_z_in_flange * offset_distance
    return C_pose



def load_workpiece_coords(folder_path):
    """
    加载最新的工件坐标数据
    
    参数:
        folder_path: JSON文件所在的文件夹路径
        
    返回:
        tuple: (delta_xyz_mm, delta_rpy_deg)
               delta_xyz_mm: 位置偏移 [x, y, z] (mm)
               delta_rpy_deg: 旋转偏移 [rx, ry, rz] (deg)
               如果加载失败，返回 (None, None)
    """
    try:

        latest_file = get_latest_json_file(folder_path)
        print(f"读取文件: {latest_file}")
            
        with open(latest_file, 'r') as f:
            data = json.load(f)
        
        A_vector = data[0]
            
        # 2. 沿着【工件坐标系】Z 轴移动的距离 a (单位：米 m)
        # 例如：0.05 代表沿工件朝向轴推进 50 毫米；-0.03 代表沿工件朝向轴退回 30 毫米
        a_meters = -0.15
        # =========================================================================

        # 直接调用封装好的外参函数获取标定数据
        camera_to_flange_pose = get_static_camera_to_flange_pose()
        
        # 获取相机到法兰的 4x4 变换矩阵 T_CF
        T_CF, _, _ = get_transform_matrix(camera_to_flange_pose)
        
        # 归一化校验（确保手动输入的四元数满足数学规范）
        norm = np.linalg.norm(A_vector[3:])
        if not np.isclose(norm, 1.0, atol=1e-3):
            print(f"⚠️ 提示: 输入的四元数未归一化 (模长为 {norm:.4f})，系统已在矩阵运算中自动纠正。")
        
        # 1. 核心转换：获得法兰坐标系下的点 B (输入为 m 和 rad，输出为 m 和 rad)
        B_vector = transform_pose(A_vector, T_CF)
        
        # 2. 沿着【工件坐标系】的 Z 轴平移：获得最终目标点
        C_vector = offset_along_workpiece_z(B_vector, a_meters)
        
        # # ========================== 打印输出结果 ==========================
        # print("\n" + "=" * 60)
        # print("【1. 输入相机坐标系下的位姿向量 A】")
        # print(f"位置 (m): X:{A_vector[0]:.6f}, Y:{A_vector[1]:.6f}, Z:{A_vector[2]:.6f}")
        # print(f"四元数 (w, x, y, z): ({A_vector[3]:.6f}, {A_vector[4]:.6f}, {A_vector[5]:.6f}, {A_vector[6]:.6f})")
        # print("-" * 60)
        
        # print("【2. 中间点：法兰坐标系下的位姿向量 B】")
        # print(f"位置 (m): X:{B_vector[0]:.6f}, Y:{B_vector[1]:.6f}, Z:{B_vector[2]:.6f}")
        # print(f"四元数 (w, x, y, z): ({B_vector[3]:.6f}, {B_vector[4]:.6f}, {B_vector[5]:.6f}, {B_vector[6]:.6f})")
        # print("-" * 60)
        
        # print(f"【3. 最终点：沿着工件 Z 轴移动 {a_meters*1000.0:.1f} mm 后的位姿向量 C】")
        # print(f"位置 (m): X:{C_vector[0]:.6f}, Y:{C_vector[1]:.6f}, Z:{C_vector[2]:.6f}")
        # print(f"四元数 (w, x, y, z): ({C_vector[3]:.6f}, {C_vector[4]:.6f}, {C_vector[5]:.6f}, {C_vector[6]:.6f})")
        # print("-" * 60)

        # 计算目标位姿与当前位姿的偏移量
        delta_xyz_mm, delta_rpy_deg = calculate_delta_T(C_vector)

        return delta_xyz_mm, delta_rpy_deg
        
    except FileNotFoundError:
        print("错误: JSON文件未找到")
        return None, None
    except json.JSONDecodeError:
        print("错误: JSON文件格式无效")
        return None, None
    except Exception as e:
        print(f"加载工件坐标失败: {e}")
        return None, None

# if __name__ == "__main__":

#     folder_path = r"E:\圆心\1"
        
#     try:
#         latest_file = get_latest_json_file(folder_path)
#         print(f"读取文件: {latest_file}")
            
#         with open(latest_file, 'r') as f:
#             data = json.load(f)
        
#         A_vector = data[0]
            
#         # 2. 沿着【工件坐标系】Z 轴移动的距离 a (单位：米 m)
#         # 例如：0.05 代表沿工件朝向轴推进 50 毫米；-0.03 代表沿工件朝向轴退回 30 毫米
#         a_meters = -0.2
#         # =========================================================================

#         # 直接调用封装好的外参函数获取标定数据
#         camera_to_flange_pose = get_static_camera_to_flange_pose()
        
#         # 获取相机到法兰的 4x4 变换矩阵 T_CF
#         T_CF, _, _ = get_transform_matrix(camera_to_flange_pose)
        
#         # 归一化校验（确保手动输入的四元数满足数学规范）
#         norm = np.linalg.norm(A_vector[3:])
#         if not np.isclose(norm, 1.0, atol=1e-3):
#             print(f"⚠️ 提示: 输入的四元数未归一化 (模长为 {norm:.4f})，系统已在矩阵运算中自动纠正。")
        
#         # 1. 核心转换：获得法兰坐标系下的点 B (输入为 m 和 rad，输出为 m 和 rad)
#         B_vector = transform_pose(A_vector, T_CF)
        
#         # 2. 沿着【工件坐标系】的 Z 轴平移：获得最终目标点 C
#         C_vector = offset_along_workpiece_z(B_vector, a_meters)
        
#         # ========================== 打印输出结果 ==========================
#         print("\n" + "=" * 60)
#         print("【1. 输入相机坐标系下的位姿向量 A】")
#         print(f"位置 (m): X:{A_vector[0]:.6f}, Y:{A_vector[1]:.6f}, Z:{A_vector[2]:.6f}")
#         print(f"四元数 (w, x, y, z): ({A_vector[3]:.6f}, {A_vector[4]:.6f}, {A_vector[5]:.6f}, {A_vector[6]:.6f})")
#         print("-" * 60)
        
#         print("【2. 中间点：法兰坐标系下的位姿向量 B】")
#         print(f"位置 (m): X:{B_vector[0]:.6f}, Y:{B_vector[1]:.6f}, Z:{B_vector[2]:.6f}")
#         print(f"四元数 (w, x, y, z): ({B_vector[3]:.6f}, {B_vector[4]:.6f}, {B_vector[5]:.6f}, {B_vector[6]:.6f})")
#         print("-" * 60)
        
#         print(f"【3. 最终点：沿着工件 Z 轴移动 {a_meters*1000.0:.1f} mm 后的位姿向量 C】")
#         print(f"位置 (m): X:{C_vector[0]:.6f}, Y:{C_vector[1]:.6f}, Z:{C_vector[2]:.6f}")
#         print(f"四元数 (w, x, y, z): ({C_vector[3]:.6f}, {C_vector[4]:.6f}, {C_vector[5]:.6f}, {C_vector[6]:.6f})")
#         print("-" * 60)
       
#         calculate_delta_T(C_vector)

#     except Exception as e:
#         print(f"错误: {e}")