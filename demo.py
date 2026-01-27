#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
# from realsenseD415 import Camera
from Aubo_Robot import Aubo_Robot


# Move robot to home pose
Aubo_Robot.initialize()
robot = Aubo_Robot(is_use_camera=True,is_use_jaw=False)
robot.go_home()


# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = robot.get_camera_data()


def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, robot, click_point_pix
        click_point_pix = (x, y)

        # Get click point in camera coordinates
        click_z = camera_depth_img[y][x] * robot.cam_depth_scale
        click_x = np.multiply(x - robot.cam_intrinsics[0][2], click_z / robot.cam_intrinsics[0][0])
        click_y = np.multiply(y - robot.cam_intrinsics[1][2], click_z / robot.cam_intrinsics[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x, click_y, click_z])
        click_point.shape = (3, 1)

        # Convert camera to robot coordinates
        # camera2robot = np.linalg.inv(robot.cam_pose)
        flange2camera = robot.cam_pose
        current_point=robot.get_current_waypoint()

        base2flange = np.eye(4)
        base2flange[:3, 3] = current_point['pos']
        rpy=robot.quaternion_to_rpy(current_point['ori'])
        base2flange[:3,:3]=robot.rpy2R(rpy)

        base2camera=  base2flange @ flange2camera

        base2obj = np.dot(base2camera[0:3, 0:3], click_point) + base2camera[0:3, 3:]  #执行变换：先旋转，再加平移

        base2obj_position = base2obj[0:3, 0]

        print(base2obj_position)
        print(base2obj_position.shape)


        flange2tool = robot.tool_pose

        base2obj = np.eye(4)
        base2obj[:3, :3] = robot.rpy2R([(180 / 360.0) * 2 * np.pi, 0, (90 / 360.0) * 2 * np.pi]) #人为指定了目标物体的姿态
        base2obj[:3,3]=base2obj_position

        base2flange = base2obj @ np.linalg.inv(flange2tool)


        target_pos = base2flange[:3,3]
        robot.plane_grasp([target_pos[0], target_pos[1], target_pos[2]])


# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    camera_color_img, camera_depth_img = robot.get_camera_data()
    # bgr_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        camera_color_img = cv2.circle(camera_color_img, click_point_pix, 7, (0, 0, 255), 2)
    cv2.imshow('color', camera_color_img)
    cv2.imshow('depth', camera_depth_img)

    if cv2.waitKey(1) == ord('c'):
        break

cv2.destroyAllWindows()
