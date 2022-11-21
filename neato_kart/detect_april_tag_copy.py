import rclpy
from rclpy.time import Time
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
#import pupil_apriltags as apriltag
import dt_apriltags as apriltag
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Transform
from .angle_helpers import quaternion_from_euler, rotation_matrix_to_euler
import numpy as np
import PyKDL

class RecordData(Node):
    def __init__(self, image_topic):
        super().__init__('ball_tracker')

        self.isImage = False
        self.isVideo = True
        self.image_name = "tilted 48 test.png"
        self.video_name = "april_tag_test3.avi"

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detector = apriltag.Detector(families="tag36h11", nthreads=2)
        self.camera_param = [971.646825, 972.962863, 501.846472, 402.829241]

        self.timestamp = None

        self.cv_image = None                        # the latest image from the camera
        self.image_num = 0                          # image frame number
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        if self.isImage:
            image_path = os.path.dirname(os.path.realpath(__file__))
            image_path = os.path.abspath(os.path.join(image_path, os.pardir))
            image_path = os.path.join(image_path, 'dataset', self.image_name)
            self.cv_image = cv2.imread(image_path)

        if self.isVideo:
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path = os.path.join(video_path, 'dataset', self.video_name)
            self.cap = cv2.VideoCapture(video_path)
            if (self.cap.isOpened() == False): 
                print("Unable to read camera feed")

        self.create_subscription(Image, image_topic, self.process_image, 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def create_map_frame(self):
        
        q = quaternion_from_euler(0, 0, 0.07)
        #test = self.tf_buffer.lookup_transform("base_link", "odom", self.timestamp)
        #print(test)
        transform = TransformStamped()
        transform.header.stamp = self.timestamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = -0.63
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.2
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        # self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.timestamp = msg.header.stamp

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        #cv2.namedWindow('undistorted_window')
        while True:
            if self.isVideo:
                ret, frame = self.cap.read()
                self.cv_image = frame
            self.run_loop()
            #self.create_map_frame()
            time.sleep(0.1)

    def run_loop(self):
        if not self.cv_image is None:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray, estimate_tag_pose = True, camera_params=self.camera_param, tag_size = 0.09)

            detected_image = self.cv_image.copy()
            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(detected_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(detected_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(detected_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(detected_image, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(detected_image, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
                tagFamily = r.tag_family.decode("utf-8")
                cv2.putText(detected_image, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print("[INFO] tag family: {}".format(tagFamily))
                print("[INFO] tag id: {}".format(r.tag_id))

                #print(r.pose_t)
                
                trans_tag_to_cam = np.matrix([[r.pose_R[0][0], r.pose_R[0][1], r.pose_R[0][2], r.pose_t[0][0]],
                                            [r.pose_R[1][0], r.pose_R[1][1], r.pose_R[1][2], r.pose_t[1][0]],
                                            [r.pose_R[2][0], r.pose_R[2][1], r.pose_R[2][2], r.pose_t[2][0]],
                                            [0, 0, 0, 1]])
                
                trans_cam_to_base = np.matrix([[0, 0, 1, 0.2],
                                                [-1, 0, 0, 0],
                                                [0, -1, 0, 0.05],
                                                [0, 0, 0, 1]])

                trans_tag_to_base = np.dot(trans_cam_to_base, trans_tag_to_cam)

                rot = PyKDL.Rotation(trans_tag_to_base[0,0], trans_tag_to_base[0,1], trans_tag_to_base[0,2],
                                    trans_tag_to_base[1,0], trans_tag_to_base[1,1], trans_tag_to_base[1,2],
                                    trans_tag_to_base[2,0], trans_tag_to_base[2,1], trans_tag_to_base[2,2])

                mat = rot.GetEulerZYX()

                print(trans_tag_to_base)
                
                print((mat[0] * 57.2958, mat[1] * 57.2958, mat[2] * 57.2958))

            cv2.imshow('video_window', detected_image)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = RecordData("/camera/image_raw")
    node.run()

def main(args=None):
    rclpy.init()
    n = RecordData("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()