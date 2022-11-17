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
import PyKDL

class RecordData(Node):
    def __init__(self, image_topic):
        super().__init__('ball_tracker')

        self.isImage = True
        self.image_name = "left 10.5 front 24.png"
        self.did_draw = False

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timestamp = None

        self.cv_image = None                        # the latest image from the camera
        self.image_num = 0                          # image frame number
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        if self.isImage:
            image_path = os.path.dirname(os.path.realpath(__file__))
            image_path = os.path.abspath(os.path.join(image_path, os.pardir))
            image_path = os.path.join(image_path, 'dataset', self.image_name)
            self.cv_image = cv2.imread(image_path)

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(Odom, image_topic, self.process_image, 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def create_map_frame(self):
        
        q = quaternion_from_euler(0, 0, 0.07)
        test = self.tf_buffer.lookup_transform("base_link", "odom", self.timestamp)
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
            self.run_loop()
            self.create_map_frame()
            time.sleep(0.5)

    def run_loop(self):
        if not self.cv_image is None and not self.did_draw:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            detector = apriltag.Detector(families="tag36h11", nthreads=2)
            camera_param = [971.646825, 972.962863, 501.846472, 402.829241]
            results = detector.detect(gray, estimate_tag_pose = True, camera_params=camera_param, tag_size = 0.09)

            detected_image = self.cv_image.copy()
            for r in results:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
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

                print(r.pose_t)
                rot = PyKDL.Rotation(r.pose_R[0][0], r.pose_R[0][1], r.pose_R[0][2],
                                    r.pose_R[1][0], r.pose_R[1][1], r.pose_R[1][2],
                                    r.pose_R[2][0], r.pose_R[2][1], r.pose_R[2][2])
                
                print(rot.GetEulerZYX())

            cv2.imshow('video_window', detected_image)
            #cv2.imshow('undistorted_window', img)
            if self.isImage:
                self.did_draw = False
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