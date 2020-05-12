import math
from collections import defaultdict

import cv2

from quadrotor_base import QuadrotorBase


class Quadrotor(QuadrotorBase):

    def __init__(self, node_name="uav", rate=30):
        super(Quadrotor, self).__init__(node_name, rate)

        self.__K_p = {
            'x': 1,
            'y': 1,
            'z': 1,
            'yaw': 1,
        }

        self.__state_name = "state"
        self.__state_topic = "/ground_truth/state"
        self.__front_cam_name = "front_cam"
        self.__front_cam_topic = "/front_cam/camera/image"
        self.__realsense_cam_name = "realsense_cam"
        self.__realsense_cam_topic = "/realsense_cam/rgb/rgb/image_raw"

        self.__enable_motors_service = "/enable_motors"

        self.__control_name = "control"
        self.__control_topic = "/cmd_vel"

    # -------------------------------------------------------------------------
    #   Control Method
    # -------------------------------------------------------------------------
    def display_realsense_cam(self):
        realsense_cam = self.get_realsense_cam()
        cv2.imshow("Front Camera", realsense_cam)
        cv2.waitKey(3)

    def go_to_target(self, target_dict, err_r):
        self.loginfo("Going to %s" % str(target_dict))

        if type(target_dict) == list:
            if not len(target_dict) == 4:
                raise ValueError("Length of target_dict must be 4 !")
            new_dict = {
                'x': target_dict[0],
                'y': target_dict[1],
                'z': target_dict[2],
                'yaw': target_dict[3],
            }
            target_dict = new_dict

        state = self.get_state()
        errs = {}

        err_square = 0
        for ax in ['x', 'y', 'z']:
            if ax in target_dict.keys():
                errs[ax] = target_dict[ax] - state[ax]
                err_square += errs[ax] ** 2
        if math.sqrt(err_square) <= err_r:
            self.loginfo("Reached target %s" % str(target_dict))
            return True

        for ax in ['roll', 'pitch', 'yaw']:
            if ax in target_dict.keys():
                errs[ax] = target_dict[ax] - state[ax]

        cmd = defaultdict(float)
        for ax in self.controllable_component():
            if ax in errs.keys():
                cmd[ax] = self.__K_p[ax] * errs[ax]
        self.send_control(cmd)
        return False

    # -------------------------------------------------------------------------
    #   State (Subscriber)
    # -------------------------------------------------------------------------
    def add_state(self, name=None, topic=None, ns=None):
        if name is None:
            name = self.__state_name
        if topic is None:
            topic = self.__state_topic
        if ns is not None:
            topic = "/" + ns + topic
        self.add_subscriber(name, topic, "Odometry")
        return

    def get_state(self):
        return self.get(self.__state_name)

    # -------------------------------------------------------------------------
    #   Front Camera (Subscriber)
    # -------------------------------------------------------------------------
    def add_front_cam(self, name=None, topic=None, ns=None):
        if name is None:
            name = self.__front_cam_name
        if topic is None:
            topic = self.__front_cam_topic
        if ns is not None:
            topic = "/" + ns + topic
        self.add_subscriber(name, topic, "Image")
        return

    def get_front_cam(self):
        return self.get(self.__front_cam_name)

    # -------------------------------------------------------------------------
    #   Realsense Camera (Subscriber)
    # -------------------------------------------------------------------------
    def add_realsense_cam(self, name=None, topic=None, ns=None):
        if name is None:
            name = self.__realsense_cam_name
        if topic is None:
            topic = self.__realsense_cam_topic
        if ns is not None:
            topic = "/" + ns + topic
        self.add_subscriber(name, topic, "Image")
        return

    def get_realsense_cam(self):
        return self.get(self.__realsense_cam_name)

    # -------------------------------------------------------------------------
    #   Enable Motors (service)
    # -------------------------------------------------------------------------
    def enable_motors(self, service_name=None, ns=None):
        if service_name is None:
            service_name = self.__enable_motors_service
        if ns is not None:
            service_name = "/" + ns + service_name
        self.call_service(service_name)
        return

    # -------------------------------------------------------------------------
    #   Control (Publisher)
    # -------------------------------------------------------------------------
    def add_control(self, name=None, topic=None, ns=None):
        if name is None:
            name = self.__control_name
        if topic is None:
            topic = self.__control_topic
        if ns is not None:
            topic = "/" + ns + topic
        self.add_publisher(name, topic, "Twist")
        return

    def send_control(self, cmd):
        return self.send(self.__control_name, cmd)
