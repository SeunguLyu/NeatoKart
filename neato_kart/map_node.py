import rclpy
from rclpy.time import Time
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import os
import dt_apriltags as apriltag
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Transform
from neato_kart.detect_april_tag import MapPoint, get_tag_2d_pose
from neato_kart.angle_helpers import euler_from_quaternion
import numpy as np
import PyKDL
import math
import json

class MapIndo():
    def __init__(self, tag_list = [], point_list = [], item_list = []):
        self.tag_list = tag_list
        self.point_list = point_list
        self.item_list = item_list

class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')

        self.map_name = "test1.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        self.tag_list = []
        self.point_list = []

        self.load_map_from_json()

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)
    
    def load_map_from_json(self):
        data = []
        with open(self.map_path) as f:
            for line in f:
                data.append(json.loads(line))

        for d in data:
            print(d["x"], d["y"], d["theta"], d["istag"])

    def run_loop(self):
        pass

if __name__ == '__main__':
    node = MapNode()
    node.run()

def main(args=None):
    rclpy.init()
    n = MapNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()