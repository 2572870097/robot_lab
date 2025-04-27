# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import quat_rotate

from .g1_amp_env_cfg import G1AmpEnvCfg
from .motions import MotionLoader


class G1AmpEnv(DirectRLEnv):
    cfg: G1AmpEnvCfg

    def __init__(self, cfg: G1AmpEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # action offset and scale
        dof_lower_limits = self.robot.data.soft_joint_pos_limits[0, :, 0]
        dof_upper_limits = self.robot.data.soft_joint_pos_limits[0, :, 1]
        self.action_offset = 0.5 * (dof_upper_limits + dof_lower_limits)
        self.action_scale = dof_upper_limits - dof_lower_limits
        # print("DOF LIMITS")
        # print(dof_lower_limits)
        # print(dof_upper_limits)

        # load motion
        self._motion_loader = MotionLoader(motion_file=self.cfg.motion_file, device=self.device)

        # DOF and key body indexes  
        # key_body_names = ["pelvis"]  
        key_body_names = [ "left_shoulder_pitch_link",
            "right_shoulder_pitch_link",
            "left_elbow_link",
            "right_elbow_link",
            "right_hip_yaw_link",
            "left_hip_yaw_link",
            "right_rubber_hand",
            "left_rubber_hand",
            "right_ankle_roll_link",
            "left_ankle_roll_link"]
        # key_body_names = [
        #     "left_hip_pitch_link",
        #     "left_hip_roll_link",
        #     "left_hip_yaw_link",
        #     "left_knee_link",
        #     "left_ankle_pitch_link",
        #     "left_ankle_roll_link",
        #     "right_hip_pitch_link",
        #     "right_hip_roll_link",
        #     "right_hip_yaw_link",
        #     "right_knee_link",
        #     "right_ankle_pitch_link",
        #     "right_ankle_roll_link",
        #     "waist_yaw_link",
        #     "waist_roll_link",
        #     "torso_link",
        #     "left_shoulder_pitch_link",
        #     "left_shoulder_roll_link",
        #     "left_shoulder_yaw_link",
        #     "left_elbow_link",
        #     "left_wrist_roll_link",
        #     "left_wrist_pitch_link",
        #     "left_wrist_yaw_link",
        #     "right_shoulder_pitch_link",
        #     "right_shoulder_roll_link",
        #     "right_shoulder_yaw_link",
        #     "right_elbow_link",
        #     "right_wrist_roll_link",
        #     "right_wrist_pitch_link",
        #     "right_wrist_yaw_link"]
        self.ref_body_index = self.robot.data.body_names.index(self.cfg.reference_body)
        self.key_body_indexes = [self.robot.data.body_names.index(name) for name in key_body_names]
        # Used to for reset strategy
        """
        | 变量名 | 含义 | 用途 |
        |--------|------|------|
        | `motion_dof_indexes` | 动作数据中与机器人关节对应的 DOF 索引 | 用于提取对应动作数据 |
        | `motion_ref_body_index` | 用于坐标对齐的参考 body 索引（例如 pelvis） | 作为 AMP 坐标变换基准 |
        | `motion_key_body_indexes` | 模仿过程中关注的关键 body 索引列表 | 用于奖励计算和模仿监督 |
        """
        self.motion_dof_indexes = self._motion_loader.get_dof_index(self.robot.data.joint_names)
        self.motion_ref_body_index = self._motion_loader.get_body_index([self.cfg.reference_body])[0]
        self.motion_key_body_indexes = self._motion_loader.get_body_index(key_body_names)
     
        # reconfigure AMP observation space according to the number of observations and create the buffer
        """
        (1)
            self.cfg.num_amp_observations
            代表我们要在判别器里一次性输入多少帧历史观测（历史窗口长度），例如 10 帧。
            self.cfg.amp_observation_space
            表示每一帧观测本身的特征维度，比如说有 30 维的状态特征。
        (2)
            gym.spaces.Box
            在 Gym/IsaacGym 中，用来描述连续型的多维观测空间。

            low=-np.inf, high=np.inf
            表示此空间每个维度都没有上下界约束（判别器之后可能会做归一化）。

            shape=(self.amp_observation_size,)
            指明这是一个一维向量，长度就是上一步算出的 amp_observation_size。

            用途
            让训练框架和判别器清楚：它接收的是一个长度为 300（举例）的实数向量。
        (3)
            amp_observation_buffer 这里构造了一个形状为 (num_envs, num_history, obs_dim) 的零张量，用来存放每个并行环境下的历史观测序列。
        """
        self.amp_observation_size = self.cfg.num_amp_observations * self.cfg.amp_observation_space
        self.amp_observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.amp_observation_size,))
        self.amp_observation_buffer = torch.zeros(
            (self.num_envs, self.cfg.num_amp_observations, self.cfg.amp_observation_space), device=self.device
        )

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        # add ground plane
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=1.0,
                    dynamic_friction=1.0,
                    restitution=0.0,
                ),
            ),
        )
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # add articulation to scene
        self.scene.articulations["robot"] = self.robot
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clone()

    def _apply_action(self):
        # self.pre_actions = self.actions.clone()
        target = self.action_offset + self.action_scale * self.actions
        self.robot.set_joint_position_target(target)

    def _get_observations(self) -> dict:
        # build task observation
        obs = compute_obs(
            self.robot.data.joint_pos,
            self.robot.data.joint_vel,
            self.robot.data.body_pos_w[:, self.ref_body_index],
            self.robot.data.body_quat_w[:, self.ref_body_index],
            self.robot.data.body_lin_vel_w[:, self.ref_body_index],
            self.robot.data.body_ang_vel_w[:, self.ref_body_index],
            self.robot.data.body_pos_w[:, self.key_body_indexes],
        )

        # update AMP observation history
        for i in reversed(range(self.cfg.num_amp_observations - 1)):
            self.amp_observation_buffer[:, i + 1] = self.amp_observation_buffer[:, i]
        # build AMP observation
        self.amp_observation_buffer[:, 0] = obs.clone()
        self.extras = {"amp_obs": self.amp_observation_buffer.view(-1, self.amp_observation_size)}

        return {"policy": obs}

    # def _get_rewards(self) -> torch.Tensor:
    #     return torch.ones((self.num_envs,), dtype=torch.float32, device=self.sim.device)
    def _get_rewards(self) -> torch.Tensor:
        total_reward, reward_log = compute_rewards(
            self.cfg.rew_termination,
            self.cfg.rew_action_l2,
            self.cfg.rew_joint_pos_limits,
            self.cfg.rew_joint_acc_l2,
            self.cfg.rew_joint_vel_l2,
            self.reset_terminated,
            self.actions,
            self.robot.data.joint_pos,
            self.robot.data.soft_joint_pos_limits,
            self.robot.data.joint_acc,
            self.robot.data.joint_vel,    
        )
        self.extras["log"] = reward_log
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        if self.cfg.early_termination:
            died = self.robot.data.body_pos_w[:, self.ref_body_index, 2] < self.cfg.termination_height
        else:
            died = torch.zeros_like(time_out)
        return died, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        self.robot.reset(env_ids)
        super()._reset_idx(env_ids)

        if self.cfg.reset_strategy == "default":
            root_state, joint_pos, joint_vel = self._reset_strategy_default(env_ids)
        elif self.cfg.reset_strategy.startswith("random"):
            start = "start" in self.cfg.reset_strategy
            root_state, joint_pos, joint_vel = self._reset_strategy_random(env_ids, start)
        else:
            raise ValueError(f"Unknown reset strategy: {self.cfg.reset_strategy}")

        self.robot.write_root_link_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_com_velocity_to_sim(root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

    # reset strategies

    def _reset_strategy_default(self, env_ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        return root_state, joint_pos, joint_vel

    def _reset_strategy_random(
        self, env_ids: torch.Tensor, start: bool = False
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        # sample random motion times (or zeros if start is True)
        num_samples = env_ids.shape[0]
        times = np.zeros(num_samples) if start else self._motion_loader.sample_times(num_samples)
        # sample random motions
        (
            dof_positions,
            dof_velocities,
            body_positions,
            body_rotations,
            body_linear_velocities,
            body_angular_velocities,
        ) = self._motion_loader.sample(num_samples=num_samples, times=times)

        # get root transforms (the humanoid torso)
        motion_torso_index = self._motion_loader.get_body_index(["pelvis"])[0]
        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, 0:3] = body_positions[:, motion_torso_index] + self.scene.env_origins[env_ids]
        root_state[:, 2] += 0.05  # lift the humanoid slightly to avoid collisions with the ground
        root_state[:, 3:7] = body_rotations[:, motion_torso_index]
        root_state[:, 7:10] = body_linear_velocities[:, motion_torso_index]
        root_state[:, 10:13] = body_angular_velocities[:, motion_torso_index]
        # get DOFs state
        dof_pos = dof_positions[:, self.motion_dof_indexes]
        dof_vel = dof_velocities[:, self.motion_dof_indexes]

        # update AMP observation
        amp_observations = self.collect_reference_motions(num_samples, times)
        self.amp_observation_buffer[env_ids] = amp_observations.view(num_samples, self.cfg.num_amp_observations, -1)

        return root_state, dof_pos, dof_vel

    # env methods

    def collect_reference_motions(self, num_samples: int, current_times: np.ndarray | None = None) -> torch.Tensor:
        # sample random motion times (or use the one specified)
        if current_times is None:
            current_times = self._motion_loader.sample_times(num_samples)
        times = (
            np.expand_dims(current_times, axis=-1)
            - self._motion_loader.dt * np.arange(0, self.cfg.num_amp_observations)
        ).flatten()
        # get motions
        (
            dof_positions,
            dof_velocities,
            body_positions,
            body_rotations,
            body_linear_velocities,
            body_angular_velocities,
        ) = self._motion_loader.sample(num_samples=num_samples, times=times)
        # compute AMP observation
        amp_observation = compute_obs(
            dof_positions[:, self.motion_dof_indexes],
            dof_velocities[:, self.motion_dof_indexes],
            body_positions[:, self.motion_ref_body_index],
            body_rotations[:, self.motion_ref_body_index],
            body_linear_velocities[:, self.motion_ref_body_index],
            body_angular_velocities[:, self.motion_ref_body_index],
            body_positions[:, self.motion_key_body_indexes],
        )
        return amp_observation.view(-1, self.amp_observation_size)


@torch.jit.script
def quaternion_to_tangent_and_normal(q: torch.Tensor) -> torch.Tensor:
    ref_tangent = torch.zeros_like(q[..., :3])
    ref_normal = torch.zeros_like(q[..., :3])
    ref_tangent[..., 0] = 1
    ref_normal[..., -1] = 1
    tangent = quat_rotate(q, ref_tangent)
    normal = quat_rotate(q, ref_normal)
    return torch.cat([tangent, normal], dim=len(tangent.shape) - 1)


@torch.jit.script
def compute_obs(
    dof_positions: torch.Tensor,
    dof_velocities: torch.Tensor,
    root_positions: torch.Tensor,
    root_rotations: torch.Tensor,
    root_linear_velocities: torch.Tensor,
    root_angular_velocities: torch.Tensor,
    key_body_positions: torch.Tensor,
) -> torch.Tensor:
    obs = torch.cat(
        (
            dof_positions,
            dof_velocities,
            root_positions[:, 2:3],  # root body height
            quaternion_to_tangent_and_normal(root_rotations),
            root_linear_velocities,
            root_angular_velocities,
            (key_body_positions - root_positions.unsqueeze(-2)).view(key_body_positions.shape[0], -1),
        ),
        dim=-1,
    )
    return obs
@torch.jit.script
def compute_rewards(
    rew_scale_termination: float,
    rew_scale_action_l2: float,
    rew_scale_joint_pos_limits: float,
    rew_scale_joint_acc_l2: float,
    rew_scale_joint_vel_l2: float,
    reset_terminated: torch.Tensor,
    actions: torch.Tensor,
    joint_pos: torch.Tensor,
    soft_joint_pos_limits: torch.Tensor,
    joint_acc: torch.Tensor,
    joint_vel: torch.Tensor,
):
    rew_termination = rew_scale_termination * reset_terminated.float()
    rew_action_l2 = rew_scale_action_l2 * torch.sum(torch.square(actions), dim=1)
    
    out_of_limits = -(joint_pos - soft_joint_pos_limits[:,:,0]).clip(max=0.0)
    out_of_limits += (joint_pos - soft_joint_pos_limits[:,:,1]).clip(min=0.0)
    rew_joint_pos_limits = rew_scale_joint_pos_limits * torch.sum(out_of_limits, dim=1)
    
    rew_joint_acc_l2 = rew_scale_joint_acc_l2 * torch.sum(torch.square(joint_acc), dim=1)
    rew_joint_vel_l2 = rew_scale_joint_vel_l2 * torch.sum(torch.square(joint_vel), dim=1)

    total_reward = rew_termination + rew_action_l2 + rew_joint_pos_limits + rew_joint_acc_l2 + rew_joint_vel_l2
    
    log = {
        "rew_termination": (rew_termination).mean(),
        "rew_action_l2": (rew_action_l2).mean(),
        "rew_joint_pos_limits": (rew_joint_pos_limits).mean(),
        "rew_joint_acc_l2": (rew_joint_acc_l2).mean(),
        "rew_joint_vel_l2": (rew_joint_vel_l2).mean(),
        }
    return total_reward, log