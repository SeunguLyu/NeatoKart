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
from visualization_msgs.msg import Marker, MarkerArray

class DriveNeato(Node):
    def __init__(self, image_topic):
        super().__init__('drive_neato')

        self.map_name = "test7.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        self.detector = apriltag.Detector(families="tag36h11", nthreads=2)
        self.camera_param = [971.646825, 972.962863, 501.846472, 402.829241]

        self.tag_required_distance = 3.0

        self.current_pose = None
        self.map_origin = None
        self.map_tag_list = []
        self.map_point_list = []
        self.map_item_list = []

        self.odom_tag_list = []
        self.odom_point_list = []
        self.odom_item_list = []

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.load_map_from_json()

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)

        self.pub_marker = self.create_publisher(MarkerArray, 'map_markers', 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_odom(self, msg):       
        translation = msg.pose.pose.position
        
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w)
        angles = euler_from_quaternion(*orientation_tuple)
        new_pose = MapPoint(translation.x, translation.y, angles[2])

        self.current_pose = new_pose

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        #cv2.setMouseCallback('video_window', self.process_mouse_event)
        while True:
            self.run_loop()
            time.sleep(0.1)

    # def process_mouse_event(self, event, x,y,flags,param):
    #     """ """
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         if (self.current_pose != None and self.map_origin == None):
    #             self.map_origin = self.current_pose
    #             print(self.map_origin.theta)
    #         elif self.map_origin != None:
    #             self.record_done = True

    def load_map_from_json(self):
        data = []
        with open(self.map_path) as f:
            for line in f:
                data.append(json.loads(line))

        for d in data:
            map_point = MapPoint()
            map_point.from_dict(d)
            if map_point.istag:
                self.map_tag_list.append(map_point)
            else:
                self.map_point_list.append(map_point)

    def run_loop(self):
        if not self.cv_image is None:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray, estimate_tag_pose = True, camera_params=self.camera_param, tag_size = 0.09)

            detected_image = self.cv_image.copy()

            for r in results:
                # check tag distance from robot, and then add when angle isn't too stiff
                current_tag = None
                for tag in self.map_tag_list:
                    if tag.tagid == r.tag_id:
                        current_tag = tag
                        break
                
                if self.current_pose != None and current_tag != None:
                    tag_to_base_pose = get_tag_2d_pose(r)
                    if tag_to_base_pose.get_distance() < self.tag_required_distance:
                        t_tag_current = tag_to_base_pose.as_matrix()
                        t_current_odom = self.current_pose.as_matrix()
                        t_tag_origin = current_tag.as_matrix()
                        t_origin_current = np.dot(t_tag_current, t_tag_origin.getI())
                        t_origin_odom = np.dot(t_current_odom, t_origin_current)
                        new_origin_pose = MapPoint()
                        new_origin_pose.from_matrix(t_origin_odom)
                        self.map_origin = new_origin_pose

                        self.update_map_in_odom()
                        break

            cv2.imshow('video_window', detected_image)
            cv2.waitKey(5)

    def update_map_in_odom(self):
        tags = []
        points = []
        for tag_pose in self.map_tag_list:
            t_tag_origin = tag_pose.as_matrix()
            t_origin_odom = self.map_origin.as_matrix()
            t_tag_odom = np.dot(t_origin_odom, t_tag_origin)
            odom_tag_pose = MapPoint(istag=True, tagid=tag_pose.tagid)
            odom_tag_pose.from_matrix(t_tag_odom)
            tags.append(odom_tag_pose)
        
        for point_pose in self.map_point_list:
            t_point_origin = point_pose.as_matrix()
            t_origin_odom = self.map_origin.as_matrix()
            t_point_odom = np.dot(t_origin_odom, t_point_origin)
            odom_point_pose = MapPoint()
            odom_point_pose.from_matrix(t_point_odom)
            points.append(odom_point_pose)

        self.odom_tag_list = tags
        self.odom_point_list = points

        #print("Updated map in odom")

        self.draw_map_in_odom()

    def draw_map_in_odom(self):
        if self.odom_point_list != None and self.odom_tag_list != None:
            marker_array = MarkerArray()
            marker_id = 0
            for tag_pose in self.odom_tag_list:
                marker = self.create_map_marker(tag_pose, marker_id)
                marker_array.markers.append(marker)
                marker_id += 1
            for point_pose in self.odom_point_list:
                marker = self.create_map_marker(point_pose, marker_id)
                marker_array.markers.append(marker)
                marker_id += 1
            
                #print("Draw map in odom")
            self.pub_marker.publish(marker_array)

    def create_map_marker(self, map_pose: MapPoint, id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.id = id

        if map_pose.istag:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.pose.position.x = map_pose.x
        marker.pose.position.y = map_pose.y
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.type = Marker.SPHERE
        return marker

if __name__ == '__main__':
    node = DriveNeato("/camera/image_raw")
    node.run()

def main(args=None):
    rclpy.init()
    n = DriveNeato("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()