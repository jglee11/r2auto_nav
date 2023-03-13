import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8, Bool, String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import scipy.stats
import numpy as np
import math
import cmath
import time



# constants
rotatechange = 0.1
speedchange = 0.1
occ_bins =
stop_distance = 0.25
scanfile = 'lidar.txt'
mapfile = 'map.txt'


# Debug flag to print output
debug = True

# Some variable to tune your bot
stopping_time_in_seconds = 600 #seconds
initial_direction = 0  # "Front", "Left", "Right", "Back"
direction_dict = {0: "Front" , 1: "Left", 2: "Right", 3: "Back"}
wall_following_dir = 1
wall_following_dir_dic = {1 : "Right", -1 : "Left"}
back_angles = range(150, 211)

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians
  
  
  class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        
         # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.target_subscription = self.create_subscription(
            Int8,
            'hot_target',
            self.target_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.target_subscription  # prevent unused variable warning

