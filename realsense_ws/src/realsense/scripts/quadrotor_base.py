import time
import math
from collections import defaultdict

import rospy
from rospy import ROSException, ServiceException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors

from cv_bridge import CvBridge  # unused CvBridgeError


class QuadrotorBase(object):

    # -------------------------------------------------------------------------
    #   Initialization
    # -------------------------------------------------------------------------
    def __init__(self, node_name="uav", rate=30):
        rospy.init_node(node_name, anonymous=True, log_level=rospy.INFO)
        self.__rate = rospy.Rate(rate)
        self.loginfo("Node \"%s\" has successfully initialized !" % node_name)

        self.__data = {}
        self.__publishers = {}

        self.__bridge = CvBridge()

        self.__supported_msg_type = {
            "Subscriber": [
                "Odometry",
                "Image",
            ],
            "Publisher": [
                "Twist",
            ],
        }

        self.__controllable_component = ['x', 'y', 'z', 'yaw']

    def add_subscriber(self, name, topic, msg_type):
        self.__data[name] = None
        try:
            rospy.Subscriber(topic, eval(msg_type), self.__callback,
                             (msg_type, name, ))
        except NameError:
            raise NameError("%s is not supported at present !" % msg_type)

        while not rospy.is_shutdown():
            if self.__data[name] is None:
                self.logwarn("Waiting for subscribing the topic " + topic)
                time.sleep(1)
                continue
            self.loginfo("Successfully subscribed %s !" % topic)
            return
        raise ROSException("Ros master has shut down !")

    def add_publisher(self, name, topic, msg_type):
        try:
            self.__publishers[name] = rospy.Publisher(topic, eval(msg_type),
                                                      queue_size=10)
        except NameError:
            raise NameError("%s is not supported at present !" % msg_type)

        return

    def call_service(self, service_name="/enable_motors"):
        self.logwarn("Waiting for service %s ..." % service_name)
        rospy.wait_for_service(service_name)
        try:
            enable_motors = rospy.ServiceProxy(service_name, EnableMotors)
            enable_motors(enable=True)
            self.loginfo("Successfully called service %s !" % service_name)
        except ServiceException as e:
            self.logerr("Service call failed: %s" % e)
        return

    # -------------------------------------------------------------------------
    #   Callback Function
    # -------------------------------------------------------------------------
    def __callback(self, data, args):
        """
        args[0]: msg_type
        args[1]: name
        """
        if eval(args[0]) == Image:
            data = self.__deal_image(data)
        elif eval(args[0]) == Odometry:
            data = self.__deal_odom(data)
        self.__data[args[1]] = data
        return

    def __deal_image(self, image):
        # try:
        #     return self.__bridge.imgmsg_to_cv2(image, "bgr8")
        # except CvBridgeError as e:
        #     self.logerr(e)
        return self.__bridge.imgmsg_to_cv2(image, "bgr8")

    def __tf_quaternion2euler(self, x, y, z, w):
        aSinInput = -2*(x*z-w*y)
        if aSinInput > 1:
            aSinInput = 1
        elif aSinInput < -1:
            aSinInput = -1

        attitude = [None] * 3
        attitude[2] = math.atan2(2.0*(x*y+w*z), w*w + x*x - y*y - z*z)
        attitude[1] = math.asin(aSinInput)
        attitude[0] = math.atan2(2.0*(y*z+w*x), w*w - x*x - y*y + z*z)
        return attitude

    def __deal_odom(self, odom):
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        attitude = self.__tf_quaternion2euler(x, y, z, w)

        state = {
            "x": odom.pose.pose.position.x,
            "y": odom.pose.pose.position.y,
            "z": odom.pose.pose.position.z,

            "row": attitude[0],
            "pitch": attitude[1],
            "yaw": attitude[2],

            "v_x": odom.twist.twist.linear.x,
            "v_y": odom.twist.twist.linear.y,
            "v_z": odom.twist.twist.linear.z,
        }
        return state

    # -------------------------------------------------------------------------
    #   Get Message
    # -------------------------------------------------------------------------
    def get(self, name):
        return self.__data[name]

    # -------------------------------------------------------------------------
    #   Send Message
    # -------------------------------------------------------------------------
    def send(self, name, cmd):
        if self.__publishers[name].data_class == Twist:
            cmd = self.__deal_twist(cmd)
        self.__publishers[name].publish(cmd)
        return

    def __deal_twist(self, cmd_dict):
        assert type(cmd_dict) == defaultdict, \
            "Error: cmd_dict is not a defaultdict"

        cmd = Twist()
        cmd.linear.x = cmd_dict['x']
        cmd.linear.y = cmd_dict['y']
        cmd.linear.z = cmd_dict['z']
        cmd.angular.z = cmd_dict['yaw']
        if 'roll' in cmd_dict.keys() or 'pitch' in cmd_dict.keys():
            self.logerr("Roll and Pitch are not supported !\n" +
                         "Please modify the code !")
        # cmd.angular.x = cmd_dict['roll']
        # cmd.angular.y = cmd_dict['pitch']
        return cmd

    def controllable_component(self):
        return self.__controllable_component

    # -------------------------------------------------------------------------
    #   Tools for ROS
    # -------------------------------------------------------------------------
    def sleep(self):
        self.__rate.sleep()
        return

    def is_shutdown(self):
        return rospy.is_shutdown()

    def loginfo(self, msg):
        return rospy.loginfo(msg)

    def logwarn(self, msg):
        return rospy.logwarn(msg)

    def logerr(self, msg):
        return rospy.logerr(msg)

    def logfatal(self, msg):
        return rospy.logfatal(msg)

    # -------------------------------------------------------------------------
    #   Tools for ROS
    # -------------------------------------------------------------------------
    def check_supported_msg_type(self):
        return str(self.__supported_msg_type)
