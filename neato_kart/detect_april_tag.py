import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy.typing
import pupil_apriltags as apriltag

class RecordData(Node):
    def __init__(self, image_topic):
        super().__init__('ball_tracker')

        self.isImage = True
        self.image_name = "left 10.5 front 72.png"
        self.did_draw = False

        self.cv_image = None                        # the latest image from the camera
        self.image_num = 0                          # image frame number
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        if self.isImage:
            image_path = os.path.dirname(os.path.realpath(__file__))
            image_path = os.path.abspath(os.path.join(image_path, os.pardir))
            image_path = os.path.join(image_path, 'dataset', self.image_name)
            self.cv_image = cv2.imread(image_path)

        self.create_subscription(Image, image_topic, self.process_image, 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        if not self.cv_image is None and not self.did_draw:
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            detector = apriltag.Detector(families="tag36h11")
            camera_info = {}
            camera_info["params"] = [313.111, 310.344, 336.114, 239.242]
            results = detector.detect(gray, estimate_tag_pose = True, camera_params=camera_info["params"], tag_size = 0.1)

            for r in results:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(self.cv_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(self.cv_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(self.cv_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(self.cv_image, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
                tagFamily = r.tag_family.decode("utf-8")
                cv2.putText(self.cv_image, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print("[INFO] tag family: {}".format(tagFamily))

                print(r.pose_err)

            cv2.imshow('video_window', self.cv_image)
            if self.isImage:
                self.did_draw = True
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