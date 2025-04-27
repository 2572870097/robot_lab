#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R, Slerp
import pinocchio as pin


# -----------------------------------------------
# 1. 四元数辅助函数
# -----------------------------------------------
def quaternion_inverse(q):
    """输入 q: (w, x, y, z)，返回其逆."""
    w, x, y, z = q
    norm_sq = w*w + x*x + y*y + z*z
    if norm_sq < 1e-8:
        norm_sq = 1e-8
    return np.array([w, -x, -y, -z], dtype=q.dtype) / norm_sq

def quaternion_multiply(q1, q2):
    """输入/输出: (w, x, y, z)"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z], dtype=q1.dtype)

def compute_angular_velocity(q_prev, q_next, dt, eps=1e-8):
    """
    根据相邻两帧四元数 (w, x, y, z) 计算角速度:
      - 相对旋转 q_rel = inv(q_prev) * q_next
      - 由 q_rel 得到旋转角度 angle 和旋转轴 axis
      - 返回 (angle / dt) * axis
    """
    q_inv = quaternion_inverse(q_prev)
    q_rel = quaternion_multiply(q_inv, q_next)
    norm_q_rel = np.linalg.norm(q_rel)
    if norm_q_rel < eps:
        return np.zeros(3, dtype=np.float32)
    q_rel /= norm_q_rel

    w = np.clip(q_rel[0], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    sin_half = np.sqrt(1.0 - w*w)
    if sin_half < eps:
        return np.zeros(3, dtype=np.float32)
    axis = q_rel[1:] / sin_half
    return (angle / dt) * axis


# -----------------------------------------------
# 2. 构造 Pinocchio RobotWrapper 的辅助函数
# -----------------------------------------------
def build_pin_robot(urdf_path, mesh_dir):
    """
    读取 URDF 文件，构造带 free-flyer 的 pin.RobotWrapper。
    参数:
        urdf_path: URDF 的路径
        mesh_dir: 关联的几何文件所在文件夹
    返回:
        robot (pin.RobotWrapper)
    """
    # 注意：若 URDF 已经包含浮动关节，这里可改用 BuildFromURDF(urdf_path, ...)
    robot = pin.RobotWrapper.BuildFromURDF(
        urdf_path,
        mesh_dir,
        pin.JointModelFreeFlyer()
    )
    return robot


# -----------------------------------------------
# 3. 主转换流程
# -----------------------------------------------
def main():
    # 3.1 读取 CSV 数据并取出需要的帧（改为帧250~550）
    csv_file = "g1/dance1_subject2.csv"
    df = pd.read_csv(csv_file, header=None)
    start_idx = 250
    end_idx = 550
    # csv_file = "g1/walk1_subject1.csv"
    # df = pd.read_csv(csv_file, header=None)
    # start_idx = 100
    # end_idx = 300
    data_orig = df.iloc[start_idx:end_idx].to_numpy(dtype=np.float32)
    N_orig = data_orig.shape[0]
    print(f"读取 CSV: {csv_file}, 帧范围[{start_idx}:{end_idx}], 共 {N_orig} 帧.")

    # 原始 CSV 前7列为 root 数据，之后为其他关节
    root_data_orig = data_orig[:, :7]      # (N_orig, 7)
    joint_data_orig = data_orig[:, 7:]       # (N_orig, D)

    # 3.2 定义原始采样率 (30fps) 和新采样率 (60fps)，构造时间序列
    fps_orig = 30
    dt_orig = 1.0 / fps_orig
    t_orig = np.linspace(0, (N_orig - 1) * dt_orig, N_orig)

    fps_new = 60
    dt_new = 1.0 / fps_new
    N_new = 2 * N_orig - 1   # 在两帧之间插入一个新帧
    t_new = np.linspace(0, (N_orig - 1) * dt_orig, N_new)

    # 3.3 对 root_data 的位置和关节角度进行插值
    # 位置（前三列）采用线性插值
    root_pos_interp = interp1d(t_orig, root_data_orig[:, 0:3], axis=0, kind='linear')(t_new)

    # 对四元数部分 (qx, qy, qz, qw) 使用 Slerp 插值
    # 注意：四元数在 CSV 中存储顺序为 (qx, qy, qz, qw)，符合 scipy 的要求
    rotations_orig = R.from_quat(root_data_orig[:, 3:7])
    slerp = Slerp(t_orig, rotations_orig)
    rotations_new = slerp(t_new)
    root_quat_interp = rotations_new.as_quat()  # (N_new, 4) 依旧为 (qx, qy, qz, qw)

    # 合并插值后的 root 数据
    root_data = np.hstack((root_pos_interp, root_quat_interp))  # (N_new, 7)

    # 对关节角度（joint_data）采用线性插值
    joint_data = interp1d(t_orig, joint_data_orig, axis=0, kind='linear')(t_new)

    # 更新帧数、采样率和时间间隔
    N = N_new
    fps = fps_new
    dt = dt_new

    # 3.4 定义关节名称 
    joint_names = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    ]
    dof_names = np.array(joint_names, dtype=np.str_)

    # 3.5 取关节位置 (不含 Root)
    dof_positions = joint_data.copy()      # shape: (N, D)

    # 3.6 计算关节速度 (中心差分 + 边界前后差分 + 高斯平滑)
    dof_velocities = np.zeros_like(dof_positions)
    dof_velocities[1:-1] = (dof_positions[2:] - dof_positions[:-2]) / (2 * dt)
    dof_velocities[0] = (dof_positions[1] - dof_positions[0]) / dt
    dof_velocities[-1] = (dof_positions[-1] - dof_positions[-2]) / dt
    dof_velocities_smoothed = gaussian_filter1d(dof_velocities, sigma=1, axis=0)

    # 3.7 指定要记录的 link 名称，并在全局坐标系下获取其位姿
    body_names = [
        "pelvis", 
        # "head_link",
        "left_shoulder_pitch_link",
        "right_shoulder_pitch_link",
        "left_elbow_link",
        "right_elbow_link",
        "right_hip_yaw_link",
        "left_hip_yaw_link",
        "right_rubber_hand",
        "left_rubber_hand",
        "right_ankle_roll_link",
        "left_ankle_roll_link"
    ]



    body_names = np.array(body_names, dtype=np.str_)
    B = len(body_names)

    body_positions = np.zeros((N, B, 3), dtype=np.float32)
    body_rotations = np.zeros((N, B, 4), dtype=np.float32)

    # 3.8 构建 pin.RobotWrapper
    #    （请将 urdf_path 和 mesh_dir 改为你自己的实际路径）
    urdf_path = "robot_description/g1/g1_29dof_rev_1_0.urdf"
    mesh_dir = "robot_description/g1"
    robot = build_pin_robot(urdf_path, mesh_dir)
    model = robot.model
    data_pk = robot.data

    nq = model.nq  # 总自由度(含free-flyer)
    if (7 + joint_data.shape[1]) != nq:
        print(f"注意: CSV 列数={7 + joint_data.shape[1]}, 但 pinocchio nq={nq}, 可能需要检查或调整脚本解析方式.")

    # 3.9 对每帧做正向运动学 (FK)，获取各 link 在世界坐标系下的姿态
    q_pin = pin.neutral(model)

    for i in range(N):
        # 设置 free-flyer
        q_pin[0:3] = root_data[i, 0:3]
        # root_data 中存储的四元数顺序为 (qx, qy, qz, qw)
        q_pin[3:7] = root_data[i, 3:7]
        # 其余关节
        dofD = joint_data.shape[1]
        q_pin[7:7 + dofD] = joint_data[i, :]

        # FK
        pin.forwardKinematics(model, data_pk, q_pin)
        pin.updateFramePlacements(model, data_pk)

        # 读取并保存各 link 的全局位姿
        for j, link_name in enumerate(body_names):
            fid = model.getFrameId(link_name)
            link_tf = data_pk.oMf[fid]  # 该 link 在世界系下的变换

            # 平移
            body_positions[i, j, :] = link_tf.translation
            # 旋转 (pin.Quaternion 默认 (x,y,z,w)，需转为 (w,x,y,z))
            quat_xyzw = pin.Quaternion(link_tf.rotation)
            body_rotations[i, j, :] = np.array([quat_xyzw.w,
                                                quat_xyzw.x,
                                                quat_xyzw.y,
                                                quat_xyzw.z],
                                               dtype=np.float32)

    # 3.10 计算 body 的线速度与角速度 (在世界坐标系下)
    # -- 线速度：中心差分 --
    body_linear_velocities = np.zeros_like(body_positions)
    body_linear_velocities[1:-1] = (body_positions[2:] - body_positions[:-2]) / (2 * dt)
    body_linear_velocities[0] = (body_positions[1] - body_positions[0]) / dt
    body_linear_velocities[-1] = (body_positions[-1] - body_positions[-2]) / dt
    body_linear_velocities = gaussian_filter1d(body_linear_velocities, sigma=1, axis=0)

    # -- 角速度：由相邻四元数计算 (世界坐标系下) --
    body_angular_velocities = np.zeros((N, B, 3), dtype=np.float32)
    for j in range(B):
        quats = body_rotations[:, j, :]
        angular_vels = np.zeros((N, 3), dtype=np.float32)
        if N > 1:
            angular_vels[0] = compute_angular_velocity(quats[0], quats[1], dt)
            angular_vels[-1] = compute_angular_velocity(quats[-2], quats[-1], dt)
        for k in range(1, N - 1):
            av1 = compute_angular_velocity(quats[k - 1], quats[k], dt)
            av2 = compute_angular_velocity(quats[k], quats[k + 1], dt)
            angular_vels[k] = 0.5 * (av1 + av2)
        # 平滑
        body_angular_velocities[:, j, :] = gaussian_filter1d(angular_vels, sigma=1, axis=0)

    # 3.11 打包保存到 NPZ
    data_dict = {
        "fps": fps,                                   # int64 标量，采样帧率
        "dof_names": dof_names,                       # unicode 数组 (D,)
        "body_names": body_names,                     # unicode 数组 (B,)
        "dof_positions": dof_positions,               # float32 (N, D)
        "dof_velocities": dof_velocities_smoothed,    # float32 (N, D)
        "body_positions": body_positions,             # float32 (N, B, 3)
        "body_rotations": body_rotations,             # float32 (N, B, 4) (w,x,y,z)
        "body_linear_velocities": body_linear_velocities,     # float32 (N, B, 3)
        "body_angular_velocities": body_angular_velocities    # float32 (N, B, 3)
    }

    out_filename = "g1.npz"
    np.savez(out_filename, **data_dict)

    print(f"已完成转换，数据保存在 {out_filename}")
    print("fps:", fps)
    print("dof_names:", dof_names.shape)
    print("body_names:", body_names.shape)
    print("dof_positions:", dof_positions.shape)
    print("dof_velocities:", dof_velocities_smoothed.shape)
    print("body_positions:", body_positions.shape)
    print("body_rotations:", body_rotations.shape)
    print("body_linear_velocities:", body_linear_velocities.shape)
    print("body_angular_velocities:", body_angular_velocities.shape)


if __name__ == "__main__":
    main()
