#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
waypoint_tool.py

用途：
1. 采集并保存机械臂当前 waypoint 到 JSON，若目标 JSON 已存在则在后面追加，不覆盖。
2. 支持对已保存点位改名。
3. 提供后续业务可直接调用的轻量封装函数：
   - 按 JSON 中点位做 move_joint / move_line / 笛卡尔移动
   - 当前位姿基础上的姿态调整
   - 两种执行器等待方式：
       a) 手动按键确认后继续
       b) 固定等待时间后继续

说明：
- 本脚本假设 robotcontrol.py 与本脚本位于同一目录，或在 Python 可导入路径中。
- 采集保存的是完整 waypoint：name / index / timestamp / joint / pos / ori
- JSON 结构为列表，便于追加与人工查看编辑。
"""

import json
import os
import time
import math
from datetime import datetime

from robotcontrol import Auboi5Robot, RobotErrorType, RobotError, logger_init


# =========================
# JSON 点位管理
# =========================

class WaypointStore:
    def __init__(self, json_path="waypoints.json"):
        self.json_path = json_path
        self.points = self._load()

    def _load(self):
        if not os.path.exists(self.json_path):
            return []
        with open(self.json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return data
        raise ValueError(f"JSON 文件格式错误，应为 list: {self.json_path}")

    def save(self):
        with open(self.json_path, "w", encoding="utf-8") as f:
            json.dump(self.points, f, ensure_ascii=False, indent=2)

    def next_index(self):
        if not self.points:
            return 1
        return max(p.get("index", 0) for p in self.points) + 1

    def exists_name(self, name):
        return any(p["name"] == name for p in self.points)

    def make_unique_name(self, base_name):
        if not self.exists_name(base_name):
            return base_name
        i = 2
        while True:
            new_name = f"{base_name}_{i}"
            if not self.exists_name(new_name):
                return new_name
            i += 1

    def add_waypoint(self, name, waypoint):
        safe_name = self.make_unique_name(name)
        record = {
            "index": self.next_index(),
            "name": safe_name,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "joint": list(waypoint["joint"]),
            "pos": list(waypoint["pos"]),
            "ori": list(waypoint["ori"]),
        }
        self.points.append(record)
        self.save()
        return record

    def list_points(self):
        return self.points

    def get_by_name(self, name):
        for p in self.points:
            if p["name"] == name:
                return p
        raise KeyError(f"未找到点位名: {name}")

    def get_by_index(self, index):
        for p in self.points:
            if p["index"] == index:
                return p
        raise KeyError(f"未找到序号: {index}")

    def rename(self, old_name, new_name):
        if self.exists_name(new_name):
            raise ValueError(f"新名称已存在: {new_name}")
        point = self.get_by_name(old_name)
        point["name"] = new_name
        self.save()

    def rename_by_index(self, index, new_name):
        if self.exists_name(new_name):
            raise ValueError(f"新名称已存在: {new_name}")
        point = self.get_by_index(index)
        point["name"] = new_name
        self.save()


# =========================
# 机械臂连接辅助
# =========================

def connect_robot(ip="localhost", port=8899):
    logger_init()
    Auboi5Robot.initialize()
    robot = Auboi5Robot()
    robot.create_context()

    result = robot.connect(ip, port)
    if result != RobotErrorType.RobotError_SUCC:
        raise RuntimeError(f"连接失败: {ip}:{port}")

    return robot


def safe_shutdown_robot(robot):
    try:
        if getattr(robot, "connected", False):
            try:
                robot.robot_shutdown()
            except Exception:
                pass
            try:
                robot.disconnect()
            except Exception:
                pass
    finally:
        Auboi5Robot.uninitialize()


# =========================
# 点位采集
# =========================

def print_waypoint_brief(point):
    print(f"[{point['index']}] {point['name']}  @ {point['timestamp']}")
    print(f"  joint = {point['joint']}")
    print(f"  pos   = {point['pos']}")
    print(f"  ori   = {point['ori']}")
    print("-" * 70)


def print_all_waypoints(store: WaypointStore):
    points = store.list_points()
    if not points:
        print("当前 JSON 中无点位。")
        return
    print("\n=== 当前已保存点位 ===")
    for p in points:
        print(f"[{p['index']}] {p['name']} | time={p['timestamp']} | pos={p['pos']}")
    print("-" * 70)


def capture_waypoints(json_path="waypoints.json", ip="localhost", port=8899):
    """
    交互式点位采集：
    - s：保存当前点位
    - l：列出已保存点位
    - r：改名
    - p：打印当前机械臂 waypoint（不保存）
    - q：退出
    """
    store = WaypointStore(json_path)

    print(f"目标 JSON: {json_path}")
    print("若文件已存在，将在后面追加点位，不覆盖原有内容。")
    print("操作说明：")
    print("  s + 回车 : 采集当前点位并保存")
    print("  l + 回车 : 列出当前 JSON 中所有点位")
    print("  r + 回车 : 修改已保存点位名字")
    print("  p + 回车 : 打印当前机械臂 waypoint（不保存）")
    print("  q + 回车 : 退出")
    print("-" * 70)

    robot = None
    try:
        robot = connect_robot(ip, port)
        robot.robot_startup()

        while True:
            cmd = input("请输入命令[s/l/r/p/q]: ").strip().lower()

            if cmd == "s":
                default_name = f"point_{store.next_index():03d}"
                name = input(f"请输入点位名（默认 {default_name}）: ").strip() or default_name

                wp = robot.get_current_waypoint()
                if wp is None:
                    print("获取当前 waypoint 失败。")
                    continue

                record = store.add_waypoint(name, wp)
                print("点位已保存：")
                print_waypoint_brief(record)

            elif cmd == "l":
                print_all_waypoints(store)

            elif cmd == "r":
                print_all_waypoints(store)
                mode = input("按名称改名输入 n，按序号改名输入 i: ").strip().lower()
                try:
                    if mode == "n":
                        old_name = input("旧名称: ").strip()
                        new_name = input("新名称: ").strip()
                        store.rename(old_name, new_name)
                        print(f"改名成功: {old_name} -> {new_name}")
                    elif mode == "i":
                        idx = int(input("序号: ").strip())
                        new_name = input("新名称: ").strip()
                        store.rename_by_index(idx, new_name)
                        print(f"改名成功: index={idx} -> {new_name}")
                    else:
                        print("无效模式。")
                except Exception as e:
                    print(f"改名失败: {e}")

            elif cmd == "p":
                wp = robot.get_current_waypoint()
                print("当前机械臂 waypoint：")
                print(json.dumps(wp, ensure_ascii=False, indent=2))

            elif cmd == "q":
                print("退出采集。")
                break

            else:
                print("无效命令。")

    except RobotError as e:
        print(f"机械臂异常: {e}")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if robot is not None:
            safe_shutdown_robot(robot)


# =========================
# 后续业务可直接调用的轻量函数
# =========================

class RobotPointExecutor:
    """
    不做步骤列表，不做复杂流程编排。
    只提供你业务中可以逐条调用的函数。
    """

    def __init__(self, robot, json_path="waypoints.json"):
        self.robot = robot
        self.store = WaypointStore(json_path)

    def reload_points(self):
        self.store = WaypointStore(self.store.json_path)

    def get_point(self, name):
        return self.store.get_by_name(name)

    def get_current_waypoint(self):
        return self.robot.get_current_waypoint()

    # ---------- 移动 ----------
    def move_joint_to(self, name, issync=True):
        p = self.get_point(name)
        joint = tuple(p["joint"])
        print(f"[MOVE_JOINT] -> {name}")
        return self.robot.move_joint(joint, issync)

    def move_line_to(self, name):
        p = self.get_point(name)
        joint = tuple(p["joint"])
        print(f"[MOVE_LINE] -> {name}")
        return self.robot.move_line(joint)

    def move_cartesian_to(self, name, rpy_deg):
        """
        用 JSON 中记录的 pos，配合你指定的 rpy_deg 做笛卡尔目标移动。
        底层实际为：笛卡尔目标 -> 逆解 -> 轴动
        """
        p = self.get_point(name)
        pos = tuple(p["pos"])
        print(f"[MOVE_CARTESIAN_TO] -> {name}, pos={pos}, rpy_deg={rpy_deg}")
        return self.robot.move_to_target_in_cartesian(pos, tuple(rpy_deg))

    def move_cartesian_abs(self, pos, rpy_deg):
        """
        直接给绝对笛卡尔位置和姿态角
        """
        print(f"[MOVE_CARTESIAN_ABS] pos={pos}, rpy_deg={rpy_deg}")
        return self.robot.move_to_target_in_cartesian(tuple(pos), tuple(rpy_deg))

    # ---------- 姿态调整 ----------
    def adjust_pose_rpy(self, delta_rpy_deg):
        """
        在当前位置上做姿态增量调整，位置保持不变。
        delta_rpy_deg: (d_rx_deg, d_ry_deg, d_rz_deg)
        """
        wp = self.robot.get_current_waypoint()
        if wp is None:
            raise RuntimeError("获取当前 waypoint 失败")

        current_joint = tuple(wp["joint"])
        current_pos = tuple(wp["pos"])
        current_ori = tuple(wp["ori"])

        current_rpy_rad = self.robot.quaternion_to_rpy(current_ori)
        if current_rpy_rad is None:
            raise RuntimeError("四元数转欧拉角失败")

        target_rpy_rad = (
            current_rpy_rad[0] + math.radians(delta_rpy_deg[0]),
            current_rpy_rad[1] + math.radians(delta_rpy_deg[1]),
            current_rpy_rad[2] + math.radians(delta_rpy_deg[2]),
        )

        target_ori = self.robot.rpy_to_quaternion(target_rpy_rad)
        if target_ori is None:
            raise RuntimeError("欧拉角转四元数失败")

        ik_result = self.robot.inverse_kin(
            joint_radian=current_joint,
            pos=current_pos,
            ori=tuple(target_ori)
        )
        if ik_result is None:
            raise RuntimeError("姿态调整逆解失败")

        print(f"[ADJUST_POSE_RPY] delta_rpy_deg={delta_rpy_deg}")
        return self.robot.move_joint(tuple(ik_result["joint"]))

    # ---------- 执行器动作后等待方式 ----------
    def actuator_then_wait_key(self, action_name="执行器动作"):
        """
        方式1：你执行完外部动作后，按回车才继续
        """
        print(f"[ACTUATOR] {action_name}")
        input("执行器动作完成后，按回车继续...")

    def actuator_then_wait_time(self, wait_sec, action_name="执行器动作"):
        """
        方式2：执行动作后，固定等待一段时间再继续
        """
        print(f"[ACTUATOR] {action_name}")
        print(f"[WAIT] 固定等待 {wait_sec:.3f} s")
        time.sleep(wait_sec)

    # ---------- 通用等待 ----------
    def wait(self, sec):
        print(f"[WAIT] {sec:.3f} s")
        time.sleep(sec)


# =========================
# 业务模板：每次移动单独调用
# =========================

def business_template(json_path="waypoints.json", ip="localhost", port=8899):
    """
    这里只给你业务模板，不做步骤列表。
    每一步都单独调用，你后面自己改顺序即可。
    """
    robot = None
    try:
        robot = connect_robot(ip, port)
        robot.robot_startup()

        # 运动参数按需调整
        robot.init_profile()
        robot.set_joint_maxacc((0.8, 0.8, 0.8, 0.8, 0.8, 0.8))
        robot.set_joint_maxvelc((0.8, 0.8, 0.8, 0.8, 0.8, 0.8))
        robot.set_end_max_line_acc(0.15)
        robot.set_end_max_line_velc(0.10)
        robot.set_end_max_angle_acc(0.20)
        robot.set_end_max_angle_velc(0.20)

        ex = RobotPointExecutor(robot, json_path=json_path)

        # 1. 回到初始化点
        ex.move_joint_to("home")
        ex.wait(0.5)

        # 2. 按记录点位做直线运动
        ex.move_line_to("approach_1")
        ex.wait(0.2)

        # 3. 到工作点
        ex.move_line_to("work_1")
        ex.wait(0.2)

        # 4. 当前位置姿态调整
        ex.adjust_pose_rpy((0.0, 0.0, 15.0))
        ex.wait(0.2)

        # 5. 执行器动作后，人工确认再继续
        ex.actuator_then_wait_key("拧紧动作（人工确认模式）")

        # 6. 再移动
        ex.move_line_to("leave_1")
        ex.wait(0.2)

        # 7. 去另一个中间点
        ex.move_joint_to("mid_safe")
        ex.wait(0.2)

        # 8. 使用 JSON 中点位的位置 + 指定姿态，做笛卡尔目标移动
        ex.move_cartesian_to("work_2", rpy_deg=(180.0, 0.0, 90.0))
        ex.wait(0.2)

        # 9. 执行器动作后，固定等待一段时间再继续
        ex.actuator_then_wait_time(1.5, "执行器动作（固定等待模式）")

        # 10. 再次移动
        ex.move_joint_to("home")

    except RobotError as e:
        print(f"机械臂异常: {e}")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if robot is not None:
            safe_shutdown_robot(robot)


# =========================
# main
# =========================

def main():
    print("请选择模式：")
    print("1. 采集点位到 JSON（追加，不覆盖）")
    print("2. 运行业务模板")
    mode = input("输入 1 / 2: ").strip()

    json_path = input("JSON 文件名（默认 waypoints.json）: ").strip() or "waypoints.json"
    ip = input("机械臂 IP（默认 localhost）: ").strip() or "localhost"
    port = int(input("端口（默认 8899）: ").strip() or "8899")

    if mode == "1":
        capture_waypoints(json_path=json_path, ip=ip, port=port)
    elif mode == "2":
        business_template(json_path=json_path, ip=ip, port=port)
    else:
        print("无效模式。")


if __name__ == "__main__":
    main()
