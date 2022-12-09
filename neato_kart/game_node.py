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
from geometry_msgs.msg import Twist, Pose2D
from enum import Enum
import json

class GameState(Enum):
    GAME_STOP = 1
    GAME_COUNT = 2
    GAME_PLAY = 3
    GAME_END = 4

class GameNode(Node):
    def __init__(self):
        super().__init__('game_node')

        self.declare_parameter('robot1_name', '')
        robot1_name = self.get_parameter('robot1_name').get_parameter_value().string_value

        self.declare_parameter('robot2_name', 'robot2')
        robot2_name = self.get_parameter('robot2_name').get_parameter_value().string_value

        self.asset_directory = os.path.dirname(os.path.realpath(__file__))
        self.asset_directory = os.path.abspath(os.path.join(self.asset_directory, os.pardir))
        self.asset_directory = os.path.join(self.asset_directory, 'assets')

        if robot1_name != "":
            robot1_name += "/"

        robot2_name += "/"

        self.create_subscription(Image, robot1_name + "processed_image", self.process_robot1_image, 10)
        self.create_subscription(Image, robot2_name + "processed_image", self.process_robot2_image, 10)

        self.create_subscription(Pose2D, robot1_name + "map_position", self.process_robot1_position, 10)

        self.pub_robot1_vel = self.create_publisher(Twist, robot1_name + 'cmd_vel', 10)
        self.pub_robot2_vel = self.create_publisher(Twist, robot2_name + 'cmd_vel', 10)

        self.cv_robot1 = None
        self.cv_robot2 = None
        self.bridge = CvBridge()

        self.game_start = False
        self.game_tick = 0

        self.map_name = "draw_map_test.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        self.map_size = 200
        self.map_multiplier = 0.0
        self.map_center_offset = (0,0)

        self.item_list = []
        self.map_tag_list = []
        self.map_point_list = []

        self.robot1_position = None

        self.load_map_from_json()

        self.game_state = GameState.GAME_STOP

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

    def process_robot1_position(self, msg):
        self.robot1_position = msg

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        pygame.init()
        pygame.mixer.pre_init(44100, -16, 2, 2048)
        pygame.mixer.init()
        pygame.mixer.music.load(os.path.join(self.asset_directory, 'sound', 'Kevin MacLeod - Pixelland.mp3'))

        pygame.mixer.music.set_volume(0.2)

        self.start_sound = pygame.mixer.Sound(os.path.join(self.asset_directory, 'sound', 'beeping.mp3'))
        self.start_sound.set_volume(0.1)

        self.image_one = pygame.image.load(os.path.join(self.asset_directory, 'images', '1.png'))
        self.image_two = pygame.image.load(os.path.join(self.asset_directory, 'images', '2.png'))
        self.image_three = pygame.image.load(os.path.join(self.asset_directory, 'images', '3.png'))
        self.image_start = pygame.image.load(os.path.join(self.asset_directory, 'images', 'start.png'))
        
        # Game Setup
        self.window_width = 1920
        self.window_height = 714
        
        self.display = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption('Neato Kart')
        while True:
            self.run_loop()
            time.sleep(0.05)

    def run_loop(self):

        self.display.fill((0,0,0))
        keys = pygame.key.get_pressed()

        if self.game_state != (GameState.GAME_COUNT or GameState.GAME_END):
            self.set_robot1_velocity(keys)
            self.set_robot2_velocity(keys)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        if not self.cv_robot1 is None:
            pygame_image = self.convert_opencv_img_to_pygame(self.cv_robot1)
            pygame_image = pygame.transform.scale(pygame_image, (952, 714))

            # #Draw image
            self.display.blit(pygame_image, (0, 0))
            self.draw_map_at_point((730,510))

        if not self.cv_robot2 is None:
            pygame_image = self.convert_opencv_img_to_pygame(self.cv_robot2)
            pygame_image = pygame.transform.scale(pygame_image, (952, 714))

            # #Draw image
            self.display.blit(pygame_image, (952 + 16, 0))
            self.draw_map_at_point((730 + 952,510))

        if self.game_state == GameState.GAME_STOP:
            if keys[pygame.K_SPACE]:
                self.game_state = GameState.GAME_COUNT
                pygame.mixer.Sound.play(self.start_sound)
                self.game_tick = pygame.time.get_ticks()       
        elif self.game_state == GameState.GAME_COUNT:
            if pygame.time.get_ticks() < self.game_tick + 500:
                pass
            elif pygame.time.get_ticks() < self.game_tick + 1500:
                image_rect = self.image_three.get_rect()
                screen_rect = self.display.get_rect()
                image_rect.center = screen_rect.center
                self.display.blit(self.image_three, image_rect)
            elif pygame.time.get_ticks() < self.game_tick + 2500:
                image_rect = self.image_two.get_rect()
                screen_rect = self.display.get_rect()
                image_rect.center = screen_rect.center
                self.display.blit(self.image_two, image_rect)
            elif pygame.time.get_ticks() < self.game_tick + 3500:
                image_rect = self.image_one.get_rect()
                screen_rect = self.display.get_rect()
                image_rect.center = screen_rect.center
                self.display.blit(self.image_one, image_rect)
            elif pygame.time.get_ticks() < self.game_tick + 4500:
                image_rect = self.image_start.get_rect()
                screen_rect = self.display.get_rect()
                image_rect.center = screen_rect.center
                self.display.blit(self.image_start, image_rect)
            else:
                pygame.mixer.music.play(-1)
                self.game_state = GameState.GAME_PLAY
                self.game_tick = pygame.time.get_ticks()
        
        
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
        if keys[pygame.K_a] and linear_vel != 0:
            ang_vel += 1.0
        if keys[pygame.K_d] and linear_vel != 0:
            ang_vel -= 1.0
        twt = Twist()
        twt.angular.z = ang_vel
        twt.linear.x = linear_vel

        self.pub_robot1_vel.publish(twt)

    def set_robot2_velocity(self, keys):
        linear_vel = 0.0
        ang_vel = 0.0

        if keys[pygame.K_UP]:
            linear_vel += 0.3
        if keys[pygame.K_DOWN]:
            linear_vel -= 0.3
        if keys[pygame.K_LEFT] and linear_vel != 0:
            ang_vel += 1.0
        if keys[pygame.K_RIGHT] and linear_vel != 0:
            ang_vel -= 1.0
        twt = Twist()
        twt.angular.z = ang_vel
        twt.linear.x = linear_vel

        self.pub_robot2_vel.publish(twt)

    def load_map_from_json(self):
        data = []
        with open(self.map_path) as f:
            for line in f:
                data.append(json.loads(line))

        low_x = 0
        low_y = 0
        high_x = 0
        high_y = 0

        for d in data:
            map_point = MapPoint()
            map_point.from_dict(d)
            if map_point.istag:
                self.map_tag_list.append(map_point)
            else:
                self.map_point_list.append(map_point)

            if map_point.x > high_y:
                high_y = map_point.x
            if map_point.x < low_y:
                low_y = map_point.x
            
            if -map_point.y > high_x:
                high_x = map_point.x
            if -map_point.y < low_x:
                low_x = map_point.x

        #print(high_x - low_x)
        #print(high_y - low_y)
        
        if high_x - low_x > high_y - low_y:
            self.map_multiplier = self.map_size/(high_x - low_x)
        else:
            self.map_multiplier = self.map_size/(high_y - low_y)

        x_offset = -low_x * self.map_multiplier - self.map_size/2.0
        y_offset = low_y * self.map_multiplier + self.map_size/2.0
        self.map_center_offset = (x_offset, y_offset)
        #print(self.map_center_offset)
    
    def draw_map_at_point(self, center):
        map_center = (center[0] + self.map_center_offset[0], center[1] + self.map_center_offset[1])

        point_list = []
        point_list.append(map_center)

        for point in self.map_point_list:
            point_x = map_center[0] - point.y * self.map_multiplier
            point_y = map_center[1] - point.x * self.map_multiplier

            point_list.append((point_x, point_y))
            pygame.draw.circle(self.display, (200, 200, 200, 200), (point_x, point_y), 7.5)

        pygame.draw.lines(self.display, (200, 200, 200, 200), False, point_list, 15)
        
        for tag in self.map_tag_list:
            point_x = map_center[0] - tag.y * self.map_multiplier
            point_y = map_center[1] - tag.x * self.map_multiplier

            pygame.draw.circle(self.display, (0,255,0), (point_x, point_y), 5.0)

        if (self.robot1_position != None):
            point_x = map_center[0] - self.robot1_position.y * self.map_multiplier
            point_y = map_center[1] - self.robot1_position.x * self.map_multiplier

            pygame.draw.circle(self.display, (0,0,255), (point_x, point_y), 10.0)

def main(args=None):
    rclpy.init()
    n = GameNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()