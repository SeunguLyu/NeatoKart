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

class CreateMap(Node):
    def __init__(self, image_topic):
        super().__init__('create_map')

        self.record_done = False

        self.isVideo = False
        self.video_name = "april_tag_test3.avi"

        self.map_name = "test1.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        self.detector = apriltag.Detector(families="tag36h11", nthreads=2)
        self.camera_param = [971.646825, 972.962863, 501.846472, 402.829241]

        self.current_pose = None
        self.map_origin = None
        self.added_tag_id = set()
        self.tag_list = []
        self.point_list = []

        self.move_distance = 0.0
        self.point_required_distance = 0.5

        self.tag_required_distance = 1.0

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        if self.isVideo:
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path = os.path.join(video_path, 'dataset', self.video_name)
            self.cap = cv2.VideoCapture(video_path)
            if (self.cap.isOpened() == False): 
                print("Unable to read camera feed")

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_odom(self, msg):
        if (self.record_done):
            return
            
        translation = msg.pose.pose.position
        
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w)
        angles = euler_from_quaternion(*orientation_tuple)
        new_pose = MapPoint(translation.x, translation.y, angles[2])

        if self.current_pose != None and self.map_origin != None:
            self.move_distance += math.dist((new_pose.x, new_pose.y), (self.current_pose.x, self.current_pose.y))
            if self.move_distance > self.point_required_distance:
                t_current_odom = new_pose.as_matrix()
                print(t_current_odom)
                t_origin_odom = self.map_origin.as_matrix()
                t_current_origin = np.dot(t_origin_odom.getI(), t_current_odom)
                print(t_current_origin)
                new_map_pose = MapPoint()
                new_map_pose.from_matrix(t_current_origin)
                self.point_list.append(new_map_pose)
                self.move_distance = 0.0
                print("new point recorded")
        self.current_pose = new_pose
        #print(self.current_pose)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        while True:
            if self.isVideo:
                ret, frame = self.cap.read()
                self.cv_image = frame
            self.run_loop()
            if self.record_done:
                self.save_map_to_json()
                cv2.destroyAllWindows()
                return
            time.sleep(0.1)

    def process_mouse_event(self, event, x,y,flags,param):
        """ """
        if event == cv2.EVENT_LBUTTONDOWN:
            if (self.current_pose != None and self.map_origin == None):
                self.map_origin = self.current_pose
                print(self.map_origin.theta)
            elif self.map_origin != None:
                self.record_done = True

    def save_map_to_json(self):
        with open(self.map_path, 'w') as outfile:
            for p in self.point_list:
                dict = p.to_dict()
                json.dump(dict, outfile)
                outfile.write("\n")
            for t in self.tag_list:
                dict = t.to_dict()
                json.dump(dict, outfile)
                outfile.write("\n")

    def run_loop(self):
        if not self.cv_image is None:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray, estimate_tag_pose = True, camera_params=self.camera_param, tag_size = 0.09)

            detected_image = self.cv_image.copy()
            for r in results:
                # check if the tag is already added, check tag distance from robot, and then add when angle isn't too stiff
                if r.tag_id not in self.added_tag_id and self.map_origin != None and self.current_pose != None:
                    tag_to_base_pose = get_tag_2d_pose(r)
                    if tag_to_base_pose.get_distance() < self.tag_required_distance and (tag_to_base_pose.theta > -2.35 and tag_to_base_pose.theta < -0.78):
                        t_tag_current = tag_to_base_pose.as_matrix()
                        t_current_odom = self.current_pose.as_matrix()
                        t_origin_odom = self.map_origin.as_matrix()
                        t_tag_odom = np.dot(t_current_odom, t_tag_current)
                        t_tag_origin = np.dot(t_origin_odom.getI(), t_tag_odom)
                        new_tag_pose = MapPoint(istag=True, tagid = r.tag_id)
                        new_tag_pose.from_matrix(t_tag_origin)
                        self.tag_list.append(new_tag_pose)
                        self.added_tag_id.add(r.tag_id)
                        print("new tag has been recorded")

            cv2.imshow('video_window', detected_image)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = CreateMap("/camera/image_raw")
    node.run()

def main(args=None):
    rclpy.init()
    n = CreateMap("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()