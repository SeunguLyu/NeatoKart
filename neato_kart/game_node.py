import rclpy
from rclpy.time import Time
from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import pygame
import sys
from neato_kart.detect_april_tag import MapPoint
from geometry_msgs.msg import Twist

class GameNode(Node):
    def __init__(self):
        super().__init__('game_node')

        self.declare_parameter('robot1_name', '')
        robot1_name = self.get_parameter('robot1_name').get_parameter_value().string_value

        self.declare_parameter('robot2_name', 'robot2')
        robot2_name = self.get_parameter('robot2_name').get_parameter_value().string_value

        if robot1_name != "":
            robot1_name += "/"

        robot2_name + "/"

        self.create_subscription(Image, robot1_name + "processed_image", self.process_robot1_image, 10)
        self.create_subscription(Image, robot2_name + "processed_image", self.process_robot1_image, 10)

        self.pub_robot1_vel = self.create_publisher(Twist, robot1_name + 'cmd_vel', 10)
        self.pub_robot2_vel = self.create_publisher(Twist, robot2_name + 'cmd_vel', 10)

        self.cv_robot1 = None
        self.cv_robot2 = None
        self.bridge = CvBridge()

        self.item_list = []

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_robot1_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_robot1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_robot2_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_robot2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        pygame.init()
        
        # Game Setup
        WINDOW_WIDTH = 1024 * 2 + 10
        WINDOW_HEIGHT = 768
        
        self.display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Neato Kart')
        while True:
            self.run_loop()
            time.sleep(0.05)

    def run_loop(self):
        keys = pygame.key.get_pressed()

        self.set_robot1_velocity(keys)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        if not self.cv_robot1 is None:
            pygame_image = self.convert_opencv_img_to_pygame(self.cv_robot1)

            # #Draw image
            self.display.blit(pygame_image, (0, 0))
            pygame.display.update()

        if not self.cv_robot1 is None:
            pygame_image = self.convert_opencv_img_to_pygame(self.cv_robot1)

            # #Draw image
            self.display.blit(pygame_image, (1024 + 10, 0))
            pygame.display.update()

    def convert_opencv_img_to_pygame(self, opencv_image):
        opencv_image = opencv_image[:,:,::-1]  #Since OpenCV is BGR and pygame is RGB, it is necessary to convert it.
        shape = opencv_image.shape[1::-1]  #OpenCV(height,width,Number of colors), Pygame(width, height)So this is also converted.
        pygame_image = pygame.image.frombuffer(opencv_image.tostring(), shape, 'RGB')

        return pygame_image
    
    def set_robot1_velocity(self, keys):
        linear_vel = 0.0
        ang_vel = 0.0

        if keys[pygame.K_w]:
            linear_vel += 0.3
        if keys[pygame.K_s]:
            linear_vel -= 0.3
        if keys[pygame.K_a]:
            ang_vel += 0.3
        if keys[pygame.K_d]:
            ang_vel -= 0.3
        twt = Twist()
        twt.angular.z = ang_vel
        twt.linear.x = linear_vel

        self.pub_robot1_vel.publish(twt)

def main(args=None):
    rclpy.init()
    n = GameNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()