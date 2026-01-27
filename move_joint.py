from robotcontrol import *
"""

机械臂轨距运动, add_waypoint()添加路点后move_track()沿路点进行运动

"""

def test_process_demo():
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # time.sleep(0.2)
        # process_get_robot_current_status = GetRobotWaypointProcess()
        # process_get_robot_current_status.daemon = True
        # process_get_robot_current_status.start()
        # time.sleep(0.2)

        queue = Queue()

        p = Process(target=runWaypoint, args=(queue,))
        p.start()
        time.sleep(5)
        print("process started.")

        # 链接服务器
        #ip = 'localhost'
        # ip = '192.168.174.128'  # 虚拟机ip
        ip = '192.168.65.100'  # 真机ip

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            robot.enable_robot_event()
            robot.init_profile()
            joint_maxvelc = (2.596177/15, 2.596177/15, 2.596177/15, 3.110177/15, 3.110177/15, 3.110177/15)
            joint_maxacc = (17.308779/5, 17.308779/5, 17.308779/5, 17.308779/5, 17.308779/5, 17.308779/5)
            robot.set_joint_maxacc(joint_maxacc)
            robot.set_joint_maxvelc(joint_maxvelc)
            robot.set_arrival_ahead_blend(0.05)
            while True:
                time.sleep(1)

                home_radian = (-90/180.0*pi, 0, -90/180.0*pi, 0, -90/180.0*pi, 0)
                robot.move_joint(home_radian, True)


                joint_radian = (-90/180.0*pi, 25/180.0*pi, -90/180.0*pi, -20/180.0*pi, -90.5/180.0*pi, 0)
                robot.move_joint(joint_radian, True)


                # joint_radian = (0, 0, 0, 0, 0, 0)
                # robot.add_waypoint(joint_radian)  # 添加路点 3
                # robot.move_joint(joint_radian, True)
                # print("-----------------------------")

                queue.put(joint_radian)

                # time.sleep(5)

                # process_get_robot_current_status.test()

                # print("-----------------------------")

                # 断开服务器链接
            robot.disconnect()

    except KeyboardInterrupt:
        robot.move_stop()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))



    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("run end-------------------------")

if __name__ == '__main__':
    test_process_demo()
    logger.info("test completed")