from dataclasses import dataclass
import numpy as np

@dataclass
class Pose:
    position: np.ndarray    # (3,) [x, y, z]  单位: m
    rotation: np.ndarray    # (3, 3) rotation matrix

@dataclass
class Twist:
    linear: np.ndarray      # (3,)  线速度 m/s
    angular: np.ndarray     # (3,)  角速度 rad/s

class TargetTracker:
    def __init__(self, dt: float):
        self.dt = dt
        self.last_pose = None
        self.last_velocity = np.zeros(3)

    def update(self, measured_pose: Pose) -> Pose:
        """
        输入: 当前时刻检测到的目标位姿（相机/基座坐标系）
        输出: 预测后的目标位姿（下一控制周期）
        """
        if self.last_pose is None:
            self.last_pose = measured_pose
            return measured_pose

        # 一阶速度估计
        v = (measured_pose.position - self.last_pose.position) / self.dt

        # 简单低通（可换卡尔曼）
        self.last_velocity = 0.7 * self.last_velocity + 0.3 * v

        # 目标预测
        predicted_position = measured_pose.position + self.last_velocity * self.dt

        self.last_pose = measured_pose

        return Pose(predicted_position, measured_pose.rotation)



class VisualServoController:
    def __init__(self, kp: float, max_linear_speed: float):
        self.kp = kp
        self.v_max = max_linear_speed

    def compute_velocity(self, current_pose: Pose, target_pose: Pose) -> Twist:
        """
        输入: 当前末端位姿 & 目标位姿
        输出: 期望末端速度 Twist
        """
        error = target_pose.position - current_pose.position

        v = self.kp * error

        # 限速
        speed = np.linalg.norm(v)
        if speed > self.v_max:
            v = v / speed * self.v_max

        return Twist(linear=v, angular=np.zeros(3))
    


class MotionPlanner:
    def __init__(self, dt: float, max_step: float):
        self.dt = dt
        self.max_step = max_step

    def integrate(self, current_pose: Pose, twist: Twist) -> Pose:
        """
        输入: 当前位姿 + 期望速度
        输出: 下一控制周期的目标位姿
        """
        delta_pos = twist.linear * self.dt

        # 防止单步位移过大（抗抖）
        step = np.linalg.norm(delta_pos)
        if step > self.max_step:
            delta_pos = delta_pos / step * self.max_step

        new_position = current_pose.position + delta_pos

        return Pose(new_position, current_pose.rotation)

class VisualServoSystem:
    def __init__(self, tracker, controller, planner, robot):
        self.tracker = TargetTracker
        self.controller = VisualServoController
        self.planner = MotionPlanner
        self.robot = robot

    def step(self, measured_target_pose: Pose):
        """
        单次控制周期
        """
        # 1. 预测目标
        target_pose = self.tracker.update(measured_target_pose)

        # # 2. 获取当前末端
        # current_pose = self.robot.get_current_waypoint()

        # # 3. 计算期望速度
        # twist = self.controller.compute_velocity(current_pose, target_pose)

        # # 4. 积分为下一目标位姿
        # next_pose = self.planner.integrate(current_pose, twist)

        # # 5. 下发位置指令
        # self.robot.set_target_pose(next_pose)

        #   1. 获取当前末端状态（真实反馈）
        current_wp = self.robot.get_current_waypoint()

        current_joint = current_wp['joint']
        current_pos   = current_wp['pos']
        current_ori   = current_wp['ori']   # 四元数 (x, y, z, w) 或 (w,x,y,z) 以你SDK为准


        # 2. 计算“期望 TCP 速度”（视觉伺服/随动核心）
        twist = self.controller.compute_velocity(
            current_pos,
            current_ori,
            target_pos,
            target_ori
        )
        # twist = [vx, vy, vz, wx, wy, wz]


        # 3. 数值积分，得到“下一微小位姿”
        dt = self.control_dt   # 例如 0.02s

        next_pos = [
            current_pos[0] + twist[0] * dt,
            current_pos[1] + twist[1] * dt,
            current_pos[2] + twist[2] * dt,
        ]

        # 姿态保持（AUBO move_line 本身就假定姿态不变）
        next_ori = current_ori


        # 4. 逆解（非常关键）
        ik_result = self.robot.inverse_kin(
            joint_radian=current_joint,
            pos=tuple(next_pos),
            ori=tuple(next_ori)
        )

        if ik_result is None:
            print("IK failed, skip this cycle")
            return

        next_joint = ik_result['joint']


        # 5. 下发直线运动指令
        self.robot.move_line(tuple(next_joint))
