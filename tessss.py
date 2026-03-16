import os
import csv
import time
import threading
from datetime import datetime
from Aubo_Robot import Aubo_Robot


def wait_for_quit(stop_event):
    """
    按键退出线程：
    在终端输入 q 再回车，即可停止采样
    """
    while not stop_event.is_set():
        cmd = input("输入 q 并回车可停止采样：").strip().lower()
        if cmd == "q":
            stop_event.set()
            print("收到退出指令，正在停止采样...")


def save_waypoint_csv(
    robot,
    duration=120.0,
    freq=5.0,
    save_dir="./outputs",
    filename=None,
):
    os.makedirs(save_dir, exist_ok=True)

    if filename is None:
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"waypoint_log_{time_str}.csv"

    csv_path = os.path.join(save_dir, filename)

    period = 1.0 / freq
    start_time = time.time()
    end_time = start_time + duration

    stop_event = threading.Event()

    # 启动按键监听线程
    key_thread = threading.Thread(target=wait_for_quit, args=(stop_event,), daemon=True)
    key_thread.start()

    f = open(csv_path, mode="w", newline="", encoding="utf-8")
    writer = csv.writer(f)

    writer.writerow([
        "sample_id",
        "timestamp",
        "datetime",
        "j1", "j2", "j3", "j4", "j5", "j6",
        "x", "y", "z",
        "raw_waypoint",
    ])
    f.flush()

    sample_id = 0
    next_time = start_time

    try:
        while time.time() < end_time and not stop_event.is_set():
            now = time.time()

            try:
                wp = robot.get_current_waypoint()

                joints = [None] * 6
                pos = [None] * 3

                if isinstance(wp, dict):
                    if "joint" in wp and wp["joint"] is not None:
                        jd = list(wp["joint"])
                        for i in range(min(6, len(jd))):
                            joints[i] = jd[i]

                    if "pos" in wp and wp["pos"] is not None:
                        pd = list(wp["pos"])
                        for i in range(min(3, len(pd))):
                            pos[i] = pd[i]

                writer.writerow([
                    sample_id,
                    now,
                    datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5],
                    pos[0], pos[1], pos[2],
                    str(wp),
                ])
                f.flush()

                print(f"[{sample_id}] {datetime.fromtimestamp(now).strftime('%H:%M:%S.%f')[:-3]}  {wp}")

            except Exception as e:
                print("read pose error:", e)

            sample_id += 1
            next_time += period
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        f.flush()
        f.close()
        print(f"CSV saved to: {csv_path}")

    return csv_path


if __name__ == "__main__":
    Aubo_Robot.initialize()
    robot = Aubo_Robot(is_use_jaw=False)

    save_waypoint_csv(
        robot=robot,
        duration=120.0,   # 最长采样时间
        freq=50.0,         # 采样频率 Hz
        save_dir="./outputs/te",
    )