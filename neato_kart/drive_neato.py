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
    ros2 run neato_kart drive_neato --ros-args -p robot_name:="robot"
    '''
    def __init__(self):
        super().__init__('drive_neato')

        self.declare_parameter('robot_name', '')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.map_name = "draw_map_test.json"
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
        self.map_track_list = []

        self.odom_tag_list = []
        self.odom_point_list = []
        self.odom_item_list = []
        self.odom_track_list = []

        self.base_tag_list = []
        self.base_point_list = []
        self.base_item_list = []
        self.base_track_list = []

        self.track = CreateTrack()

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.load_map_from_json()

        if robot_name != "":
            robot_name += "/"

        self.create_subscription(Image, robot_name + "camera/image_raw", self.process_image, 10)
        self.create_subscription(Odometry, robot_name + "odom", self.process_odom, 10)

        self.pub_marker = self.create_publisher(MarkerArray, robot_name + 'map_markers', 10)
        self.pub_image = self.create_publisher(Image, robot_name + "processed_image", 10)

        self.pub_marker = self.create_publisher(MarkerArray, robot_name + 'map_markers', 10)
        self.track_marker = self.create_publisher(MarkerArray, robot_name + 'track_markers', 10)
        
        self.pub_position_in_map = self.create_publisher(Pose2D, robot_name + "map_position", 10)

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
        #cv2.namedWindow('video_window')
        #cv2.setMouseCallback('video_window', self.process_mouse_event)
        while True:
            self.run_loop()
            if (self.odom_point_list != None):
                self.update_map_in_base()
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

            # detected_image = self.track.draw_track((self.base_tag_list + self.base_point_list), detected_image)
            detected_image = self.track.draw_track(self.base_point_list, detected_image)

            for r in results:
                # check tag distance from robot, and then add when angle isn't too stiff
                draw_apriltag(detected_image, r)
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

            #cv2.imshow('video_window', detected_image)
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(detected_image, "bgr8"))
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
        # print("Point: ")
        for point_pose in self.map_point_list:
            # print(point_pose.x, point_pose.y, math.degrees(point_pose.theta))
            t_point_origin = point_pose.as_matrix()
            t_origin_odom = self.map_origin.as_matrix()
            t_point_odom = np.dot(t_origin_odom, t_point_origin)
            odom_point_pose = MapPoint()
            odom_point_pose.from_matrix(t_point_odom)
            points.append(odom_point_pose)
        # print("Point end")
        self.odom_tag_list = tags
        self.odom_point_list = points

        track_points = []
        self.map_track_list = self.track.generate_track(0.2, self.map_tag_list + self.map_point_list)
        # print("Track: ")
        for track_pose in self.map_track_list:
            # print(track_pose.x, track_pose.y, math.degrees(track_pose.theta))
            t_track_origin = track_pose.as_matrix()
            t_origin_odom = self.map_origin.as_matrix()
            t_track_odom = np.dot(t_origin_odom, t_track_origin)
            odom_point_pose = MapPoint()
            odom_point_pose.from_matrix(t_track_odom)
            track_points.append(odom_point_pose)
        # print("Track end")
        self.odom_track_list = track_points

        #print("Updated map in odom")

        self.draw_map_in_odom()
        self.draw_track_in_odom()

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

    def draw_track_in_odom(self):
        if self.odom_track_list != None:
            marker_array = MarkerArray()
            marker_id = 0
            for pose in self.odom_track_list:
                marker = self.create_track_marker(pose, marker_id)
                marker_array.markers.append(marker)
                marker_id += 1
            
                #print("Draw map in odom")
            self.track_marker.publish(marker_array)

    def update_map_in_base(self):
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
            #print(base_point_pose)

        self.base_tag_list = tags
        self.base_point_list = points

    def create_track_marker(self, track_point: TrackPoint, id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.id = id

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose.position.x = track_point.x
        marker.pose.position.y = track_point.y
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.type = Marker.SPHERE
        return marker

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

def main(args=None):
    rclpy.init()
    n = DriveNeato()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()