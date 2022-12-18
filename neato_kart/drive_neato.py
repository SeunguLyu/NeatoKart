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
from geometry_msgs.msg import PoseStamped, Pose2D, Point, Quaternion, TransformStamped, Transform
from neato_kart.detect_april_tag import MapPoint, get_tag_2d_pose, draw_apriltag
from neato_kart.angle_helpers import euler_from_quaternion
import numpy as np
import math
import json
from visualization_msgs.msg import Marker, MarkerArray
from neato_kart.create_track import TrackPoint, CreateTrack

class DriveNeato(Node):
    '''
    This node runs april tag detection and localize Neato inside the map frame 
    through multiple transformation matrix.

    To Run: ros2 run neato_kart drive_neato --ros-args -p robot_name:="ROBOT_NAME"
    '''
    def __init__(self):
        super().__init__('drive_neato')

        # define parameter for multiple robot control
        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # define path for the map that is being used
        self.map_name = "test5.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        # create april tag detector, define camera parameters
        self.detector = apriltag.Detector(families="tag36h11", nthreads=2)
        self.camera_param = [971.646825, 972.962863, 501.846472, 402.829241]

        # the distance between april tag and Neato to update the map's origin in the odom frame
        self.tag_required_distance = 2.0

        # save pose in different frames
        self.current_pose = None

        self.map_origin = None
        self.map_tag_list = []
        self.map_point_list = []

        self.odom_tag_list = []
        self.odom_point_list = []

        self.base_tag_list = []
        self.base_point_list = []

        # CreateTrack object for track generation
        self.track = CreateTrack()

        # image related
        self.cv_image = None
        self.bridge = CvBridge()

        # load map from JSON file and save as MapPoint
        self.load_map_from_json()

        # if parameter is defined, add the robot name to the topics
        if robot_name != "":
            robot_name += "/"

        # subscribe to Neato's camera
        self.create_subscription(Image, robot_name + "camera/image_raw", self.process_image, 10)

        # subscribe to Neato's Odom, get robot's position in the odom frame
        self.create_subscription(Odometry, robot_name + "odom", self.process_odom, 10)

        # publish processed image (with track drawing) to be used in the game node
        self.pub_image = self.create_publisher(Image, robot_name + "processed_image", 10)

        # publish robot's position in the map to be used in the game node
        self.pub_position_in_map = self.create_publisher(Pose2D, robot_name + "map_position", 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_odom(self, msg):
        """ Process robot's current position in the Odom frame.
            If map origin is defined, get robot's position in the Map frame 
            and publish the pose"""       
        # convert msg into Neato's 2D position in Odom frame
        translation = msg.pose.pose.position
        
        orientation_tuple = (msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w)
        angles = euler_from_quaternion(*orientation_tuple)
        new_pose = MapPoint(translation.x, translation.y, angles[2])

        self.current_pose = new_pose

        # if map origin is defined, convert pose in Odom to pose in Map and publish it
        if (self.map_origin != None):
            t_base_odom = self.current_pose.as_matrix()
            t_origin_odom = self.map_origin.as_matrix()
            t_base_origin = np.dot(t_origin_odom.getI(), t_base_odom)
            pose = MapPoint()
            pose.from_matrix(t_base_origin)

            final_pose = Pose2D()
            final_pose.x = pose.x
            final_pose.y = pose.y
            final_pose.theta = pose.theta
            self.pub_position_in_map.publish(final_pose)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            if (self.odom_point_list != None):
                self.update_map_in_base()

    def load_map_from_json(self):
        """ This function loads saved map from JSON and convert each tag/points
            into MapPoint object. """
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
        """ This function detects April Tag and update the position of the map frame's
            origin. This function also draws track on the OpenCV image and publish it 
            so that it can be used inside the game node """
        if not self.cv_image is None:
            # detect April Tags
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray, estimate_tag_pose = True, camera_params=self.camera_param, tag_size = 0.093)

            detected_image = self.cv_image.copy()

            # draw track on the OpenCV image
            detected_image = self.track.draw_track(self.base_point_list, detected_image)

            for r in results:
                draw_apriltag(detected_image, r)
                current_tag = None
                # only detect tags that are availble in the map
                for tag in self.map_tag_list:
                    if tag.tagid == r.tag_id:
                        current_tag = tag
                        break
                
                # compute tag's position in the Odom frame, and based on that 
                # get the position of map's origin in the Odom frame
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
            
            # publish the image so that it can be used in game node
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(detected_image, "bgr8"))

    def update_map_in_odom(self):
        """ This function computes position of the map's tag/point in Odom frame
            based on the map's origin position in Odom frame """
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

    def update_map_in_base(self):
        """ This function computes position of the map's tag/point in Neato's base frame
            based on the tag/point position in the Odom frame. This is needed for track
            generation."""
        points = []
        tags = []
        for tag_pose in self.odom_tag_list:
            t_tag_odom = tag_pose.as_matrix()
            t_base_odom = self.current_pose.as_matrix()
            t_tag_base = np.dot(t_base_odom.getI(), t_tag_odom)
            base_tag_pose = MapPoint(istag=True, tagid=tag_pose.tagid)
            base_tag_pose.from_matrix(t_tag_base)
            tags.append(base_tag_pose)
        
        for point_pose in self.odom_point_list:
            t_point_odom = point_pose.as_matrix()
            t_base_odom = self.current_pose.as_matrix()
            t_point_base = np.dot(t_base_odom.getI(), t_point_odom)
            base_point_pose = MapPoint()
            base_point_pose.from_matrix(t_point_base)
            points.append(base_point_pose)

        self.base_tag_list = tags
        self.base_point_list = points

def main(args=None):
    rclpy.init()
    n = DriveNeato()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()