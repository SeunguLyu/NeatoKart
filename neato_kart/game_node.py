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
import numpy as np
import random
import math

class GameState(Enum):
    GAME_TITLE = 1
    GAME_STOP = 2
    GAME_COUNT = 3
    GAME_PLAY = 4
    GAME_END = 5

class ItemType(Enum):
    BANANA = 1
    TURTLE = 2
    BOOST = 3

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
        self.create_subscription(Pose2D, robot2_name + "map_position", self.process_robot2_position, 10)

        self.pub_robot1_vel = self.create_publisher(Twist, robot1_name + 'cmd_vel', 10)
        self.pub_robot2_vel = self.create_publisher(Twist, robot2_name + 'cmd_vel', 10)

        self.cv_robot1 = None
        self.cv_robot2 = None
        self.bridge = CvBridge()

        self.game_start = False
        self.game_tick = 0

        # UI related
        self.p2_offset = 952 + 16
        self.window_width = 1920
        self.window_height = 714

        self.title_tick = 0

        # Minimap Related
        self.map_name = "example_map.json"
        self.map_path = os.path.dirname(os.path.realpath(__file__))
        self.map_path = os.path.abspath(os.path.join(self.map_path, os.pardir))
        self.map_path = os.path.join(self.map_path, 'maps', self.map_name)

        self.map_size = 200
        self.map_multiplier = 0.0
        self.map_center_offset = (0,0)
        self.map_boundary = [0.0, 0.0, 0.0, 0.0]

        self.map_tag_list = []
        self.map_point_list = []

        # Game Related
        self.normal_lin_speed = 0.2
        self.boost_lin_speed = 0.3
        self.ang_speed = 1.0

        # Item Related
        self.robot_item = [None, None]

        self.robot_is_rotating = [False, False]
        self.robot_rotate_tick = [0, 0]

        self.robot_on_boost = [False, False]
        self.robot_boost_tick = [0, 0]
        self.banana_list = []
        self.turtle_list = []

        # Race Progress Related
        self.robot_position = [None, None]
        self.robot_current_tag = [0, 0]
        self.robot_total_tag = [0, 0]

        self.load_map_from_json()

        self.game_state = GameState.GAME_TITLE

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
        pose = MapPoint()
        pose.x = msg.x
        pose.y = msg.y
        pose.theta = msg.theta
        self.robot_position[0] = pose

    def process_robot2_position(self, msg):
        pose = MapPoint()
        pose.x = msg.x
        pose.y = msg.y
        pose.theta = msg.theta
        self.robot_position[1] = pose

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        pygame.init()
        pygame.mixer.pre_init(44100, -16, 2, 2048)
        pygame.mixer.init()
        pygame.mixer.music.load(os.path.join(self.asset_directory, 'sound', 'Kevin MacLeod - Pixelland.mp3'))

        pygame.mixer.music.set_volume(0.35)

        self.start_sound = pygame.mixer.Sound(os.path.join(self.asset_directory, 'sound', 'beeping.mp3'))
        self.start_sound.set_volume(0.25)
        
        self.image_background = pygame.image.load(os.path.join(self.asset_directory, 'images', 'background.png'))
        self.image_title = pygame.image.load(os.path.join(self.asset_directory, 'images', 'neatokart.png'))
        self.image_neato = pygame.image.load(os.path.join(self.asset_directory, 'images', 'neato.png'))
        self.image_press = pygame.image.load(os.path.join(self.asset_directory, 'images', 'press.png'))

        self.image_one = pygame.image.load(os.path.join(self.asset_directory, 'images', '1.png'))
        self.image_two = pygame.image.load(os.path.join(self.asset_directory, 'images', '2.png'))
        self.image_three = pygame.image.load(os.path.join(self.asset_directory, 'images', '3.png'))
        self.image_start = pygame.image.load(os.path.join(self.asset_directory, 'images', 'start.png'))
        self.image_banana = pygame.image.load(os.path.join(self.asset_directory, 'images', 'banana.png'))
        self.image_boost = pygame.image.load(os.path.join(self.asset_directory, 'images', 'mushroom.png'))
        self.image_turtle = pygame.image.load(os.path.join(self.asset_directory, 'images', 'turtle.png'))
        self.image_end = pygame.image.load(os.path.join(self.asset_directory, 'images', 'victory.png'))

        self.display = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption('Neato Kart')
        while True:
            self.run_loop()
            #time.sleep(0.05)
            pygame.time.wait(66)

    def run_loop(self):
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        if self.game_state == GameState.GAME_TITLE:
            if keys[pygame.K_SPACE]:
                self.game_state = GameState.GAME_STOP
                self.title_tick = pygame.time.get_ticks()
        elif self.game_state == GameState.GAME_STOP:
            if keys[pygame.K_SPACE] and self.title_tick + 500 < pygame.time.get_ticks():
                self.game_state = GameState.GAME_COUNT
                pygame.mixer.Sound.play(self.start_sound)
                self.game_tick = pygame.time.get_ticks()          

        for turtle in self.turtle_list:
            turtle_offset = np.matrix([[0.05],[0],[1]])
            turtle_pose = np.dot(turtle.as_matrix(), turtle_offset)
            turtle.x = turtle_pose[0,0]
            turtle.y = turtle_pose[1,0]

            if (turtle.x > self.map_boundary[0] + 1 
            or turtle.x < self.map_boundary[1] - 1 
            or turtle.y > self.map_boundary[2] + 1 
            or turtle.y < self.map_boundary[3] - 1):
                self.turtle_list.remove(turtle)

        if self.game_state == GameState.GAME_STOP or self.game_state == GameState.GAME_PLAY:
            self.set_robot_control(keys, 0)
            self.set_robot_control(keys, 1)
        elif self.game_state == GameState.GAME_END:
            twt = Twist()
            twt.linear.x = 0.0
            twt.angular.z = 0.0
            self.pub_robot1_vel.publish(twt)
            self.pub_robot2_vel.publish(twt)
        
        self.display.fill((0,0,0))
        if not self.cv_robot1 is None:
            self.draw_robot_ui(0)
        if not self.cv_robot2 is None:
            self.draw_robot_ui(1)
        self.draw_game_global()
        pygame.display.update()

    def draw_game_global(self):
        if self.game_state == GameState.GAME_TITLE:
            self.display.blit(self.image_background, (0,0))

            image_rect = self.image_title.get_rect()
            screen_rect = self.display.get_rect()
            image_rect.center = screen_rect.center
            image_rect.centery -= 240
            self.display.blit(self.image_title, image_rect)

            neato = pygame.transform.scale(self.image_neato, (350, 350))
            image_rect = neato.get_rect()
            screen_rect = self.display.get_rect()
            image_rect.center = screen_rect.center
            image_rect.centery -= 20
            self.display.blit(neato, image_rect)

            if pygame.time.get_ticks() < self.title_tick + 1000:
                image_rect = self.image_press.get_rect()
                screen_rect = self.display.get_rect()
                image_rect.center = screen_rect.center
                image_rect.centery += 220
                self.display.blit(self.image_press, image_rect)
            elif pygame.time.get_ticks() < self.title_tick + 2000:
                pass
            else:
                self.title_tick = pygame.time.get_ticks()

        elif self.game_state == GameState.GAME_STOP:
            pass    
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
        elif self.game_state == GameState.GAME_END:
            image_rect = self.image_end.get_rect()
            screen_rect = self.display.get_rect()
            image_rect.center = screen_rect.center
            self.display.blit(self.image_end, image_rect)
    
    def draw_robot_ui(self, robot_index):
        i = robot_index
        offset = 0
        robot_image = None
        robot_pose = self.robot_position[i]
        if i == 0:
            robot_image = self.cv_robot1
        else:
            robot_image = self.cv_robot2
            offset += self.p2_offset

        # convert opencv image to pygame image
        pygame_image = self.convert_opencv_img_to_pygame(robot_image)
        pygame_image = pygame.transform.scale(pygame_image, (952, 714))

        # Draw image
        self.display.blit(pygame_image, (offset, 0))

        # draw items
        for banana in self.banana_list:
            dist = self.distance_from_pose(banana[0], banana[1], i)
            banana_base_pose = np.dot(robot_pose.as_matrix().getI(), np.matrix([[banana[0]],[banana[1]],[1]]))
            if banana_base_pose[0,0] > 0.2:
                banana_pixel = self.position_to_img_pixel(banana_base_pose[0,0], banana_base_pose[1,0])
                if dist > 3.0:
                    continue  
                elif dist > 2.0:
                    size = 32
                else:
                    size = 192 - 80 * dist
                banana_resized = pygame.transform.scale(self.image_banana, (size, size))
                if (banana_pixel[0]-size/2 < 0) or (banana_pixel[0]-size/2 > 952):
                    pass
                else:
                    self.display.blit(banana_resized, (offset+banana_pixel[0]-size/2, banana_pixel[1]-size/2))
        
        for turtle in self.turtle_list:
            dist = self.distance_from_pose(turtle.x, turtle.y, i)
            turtle_base_pose = np.dot(robot_pose.as_matrix().getI(), np.matrix([[turtle.x],[turtle.y],[1]]))
            if turtle_base_pose[0,0] > 0.2:
                turtle_pixel = self.position_to_img_pixel(turtle_base_pose[0,0], turtle_base_pose[1,0])
                if dist > 3.0:
                    continue  
                elif dist > 2.0:
                    size = 32
                else:
                    size = 192 - 80 * dist
                turtle_resized = pygame.transform.scale(self.image_turtle, (size, size))
                if (turtle_pixel[0]-size/2 < 0) or (turtle_pixel[0]-size/2 > 952):
                    pass
                else:
                    self.display.blit(turtle_resized, (offset+turtle_pixel[0]-size/2, turtle_pixel[1]-size/2))

        # draw map
        self.draw_map_at_point((820 + offset,580))

        # draw item UI
        item_ui_origin = (10 + offset, 10)
        pygame.draw.lines(self.display, (200, 200, 200, 200), True, 
                            [item_ui_origin, 
                            (item_ui_origin[0] + 100, item_ui_origin[1]), 
                            (item_ui_origin[0] + 100, item_ui_origin[1] + 100),
                            (item_ui_origin[0], item_ui_origin[1] + 100)]
                            , 5)
        
        if self.robot_item[i] == ItemType.BANANA:
            banana_resized = pygame.transform.scale(self.image_banana, (80, 80))
            self.display.blit(banana_resized, (item_ui_origin[0] + 10, item_ui_origin[1] + 10))
        elif self.robot_item[i] == ItemType.TURTLE:
            turtle_resized = pygame.transform.scale(self.image_turtle, (80, 80))
            self.display.blit(turtle_resized, (item_ui_origin[0] + 10, item_ui_origin[1] + 10))
        elif self.robot_item[i] == ItemType.BOOST:
            boost_resized = pygame.transform.scale(self.image_boost, (80, 80))
            self.display.blit(boost_resized, (item_ui_origin[0] + 10, item_ui_origin[1] + 10))

        # draw checkpoint progress
        circle_distance = 60
        total_tag = len(self.map_tag_list) + 1
        start_point = [offset + 476 - (total_tag-1)*circle_distance/2, 60]
        for j in range(total_tag):
            width = 5
            if self.robot_total_tag[i] >= j + 1:
                width = 0
            pygame.draw.circle(self.display, (79, 238, 77, 255), start_point, 20, width)
            start_point[0] += circle_distance

    def convert_opencv_img_to_pygame(self, opencv_image):
        opencv_image = opencv_image[:,:,::-1]  #Since OpenCV is BGR and pygame is RGB, it is necessary to convert it.
        shape = opencv_image.shape[1::-1]  #OpenCV(height,width,Number of colors), Pygame(width, height)So this is also converted.
        pygame_image = pygame.image.frombuffer(opencv_image.tostring(), shape, 'RGB')

        return pygame_image
    
    def position_to_img_pixel(self, x, y, z=0):
        t_cam_base = np.matrix([[0, 0, 1, 0.2],
                                [-1, 0, 0, 0],
                                [0, -1, 0, 0.05],
                                [0, 0, 0, 1]])

        # camera intrinsics
        K = np.matrix([[971.646825, 0.000000, 501.846472], 
                        [0.000000, 972.962863, 402.829241], 
                        [0.000000, 0.000000, 1.000000]])
        
        pose_in_cam = np.dot(t_cam_base.getI(), np.matrix([[x],[y],[z],[1]]))

        pixel_in_img = np.dot(K, np.matrix([[pose_in_cam[0,0]],[pose_in_cam[1,0]],[pose_in_cam[2,0]]]))

        return (pixel_in_img[0,0]/pixel_in_img[2,0], pixel_in_img[1,0]/pixel_in_img[2,0])
    
    def distance_from_pose(self, x, y, robot_index):
        robot_pose = self.robot_position[robot_index]

        if robot_pose == None:
            return 10000.0
        point1 = np.array((x, y))
        point2 = np.array((robot_pose.x, robot_pose.y))

        dist = np.linalg.norm(point1 - point2)

        return dist
    
    def set_robot_control(self, keys, robot_index):
        i = robot_index
        key_list = []
        if i == 0:
            key_list = [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_LSHIFT]
        else:
            key_list = [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_RSHIFT]
        robot_pose = self.robot_position[i]
        speed = self.normal_lin_speed
        # boost check
        if self.robot_on_boost[i]:
            if pygame.time.get_ticks() > self.robot_boost_tick[i] + 5000:
                self.robot_on_boost[i] = False
            else:
                speed = self.boost_lin_speed

        # neato speed related
        linear_vel = 0.0
        ang_vel = 0.0

        if self.robot_is_rotating[i]:
            ang_vel = 2.0
            if pygame.time.get_ticks() > self.robot_rotate_tick[i] + 3500:
                self.robot_is_rotating[i] = False
        else:
            if keys[key_list[0]]:
                linear_vel += speed
            if keys[key_list[1]]:
                linear_vel -= speed
            if keys[key_list[2]] and linear_vel != 0:
                ang_vel += self.ang_speed
            if keys[key_list[3]] and linear_vel != 0:
                ang_vel -= self.ang_speed
        twt = Twist()
        twt.angular.z = ang_vel
        twt.linear.x = linear_vel

        if i == 0:
            self.pub_robot1_vel.publish(twt)
        else:
            self.pub_robot2_vel.publish(twt)

        if self.game_state == GameState.GAME_STOP:
            return

        # checkpoint check
        next_point = self.map_tag_list[self.robot_current_tag[i]]
        checkpoint_dist = self.distance_from_pose(next_point.x, next_point.y, i)
        if checkpoint_dist < 0.4:
            self.robot_current_tag[i] += 1
            self.robot_total_tag[i] += 1
            if self.robot_current_tag[i] == len(self.map_tag_list):
                self.robot_current_tag[i] = 0
            elif self.robot_total_tag[i] == len(self.map_tag_list) + 1:
                self.game_state = GameState.GAME_END
            if self.robot_item[i] == None:
                self.robot_item[i] = random.choice(list(ItemType))
                #self.robot1_item = ItemType.TURTLE

        # item related
        if self.robot_is_rotating[i] == False:
            # using item
            if keys[key_list[4]]:
                if self.robot_item[i] == ItemType.BANANA:
                    self.robot_item[i] = None
                    banana_offset = None
                    if keys[key_list[1]]:
                        banana_offset = np.matrix([[-0.3],[0],[1]])
                    else:
                        banana_offset = np.matrix([[1.0],[0],[1]])

                    banana_pose = np.dot(robot_pose.as_matrix(), banana_offset)
                    self.banana_list.append((banana_pose[0,0], banana_pose[1,0]))
                
                elif self.robot_item[i] == ItemType.TURTLE:
                    self.robot_item[i] = None
                    turtle_offset = None
                    if keys[key_list[1]]:
                        turtle_offset = np.matrix([[-0.3],[0],[1]])
                        turtle_angle = robot_pose.theta
                        turtle_angle += math.pi
                        if (turtle_angle) >= math.pi * 2:
                            turtle_angle -= math.pi * 2
                    else:
                        turtle_offset = np.matrix([[0.3],[0],[1]])
                        turtle_angle = robot_pose.theta
                    turtle_pose = np.dot(robot_pose.as_matrix(), turtle_offset)
                    turtle_point = MapPoint(turtle_pose[0,0], turtle_pose[1,0], turtle_angle)
                    self.turtle_list.append(turtle_point)
                
                elif self.robot_item[i] == ItemType.BOOST:
                    self.robot_item[i] = None
                    self.robot_on_boost[i] = True
                    self.robot_boost_tick[i] = pygame.time.get_ticks()

            # detect if robot is colliding with items
            for banana in self.banana_list:
                dist = self.distance_from_pose(banana[0], banana[1], i)
                if dist < 0.1:
                    self.robot_is_rotating[i] = True
                    self.banana_list.remove(banana)
                    self.robot_rotate_tick[i] = pygame.time.get_ticks()
                    break
            
            for turtle in self.turtle_list:
                dist = self.distance_from_pose(turtle.x, turtle.y, i)
                if dist < 0.2:
                    self.robot_is_rotating[i] = True
                    self.turtle_list.remove(turtle)
                    self.robot_rotate_tick[i] = pygame.time.get_ticks()
                    break
    
    def draw_map_at_point(self, center):
        map_half = self.map_size/2 + 20
        pygame.draw.lines(self.display, (200, 200, 200, 200), True, 
                                [(center[0] - map_half, center[1] - map_half),
                                (center[0] + map_half, center[1] - map_half), 
                                (center[0] + map_half, center[1] + map_half),
                                (center[0] - map_half, center[1] + map_half)]
                                , 5)

        map_center = (center[0] + self.map_center_offset[0], center[1] + self.map_center_offset[1])

        point_list = []
        point_list.append(map_center)

        for point in self.map_point_list:
            point_x = map_center[0] - point.y * self.map_multiplier
            point_y = map_center[1] - point.x * self.map_multiplier

            point_list.append((point_x, point_y))
            pygame.draw.circle(self.display, (200, 200, 200, 200), (point_x, point_y), 7.5)

        pygame.draw.lines(self.display, (200, 200, 200, 200), False, point_list, 15)

        for banana in self.banana_list:
            point_x = map_center[0] - banana[1] * self.map_multiplier
            point_y = map_center[1] - banana[0] * self.map_multiplier

            pygame.draw.circle(self.display, (255,255,0), (point_x, point_y), 5.0)
        
        for turtle in self.turtle_list:
            point_x = map_center[0] - turtle.y * self.map_multiplier
            point_y = map_center[1] - turtle.x * self.map_multiplier

            pygame.draw.circle(self.display, (128,128,0), (point_x, point_y), 5.0)
        
        for tag in self.map_tag_list:
            point_x = map_center[0] - tag.y * self.map_multiplier
            point_y = map_center[1] - tag.x * self.map_multiplier

            pygame.draw.circle(self.display, (0,255,0), (point_x, point_y), 5.0)

        if (self.robot_position[0] != None):
            point_x = map_center[0] - self.robot_position[0].y * self.map_multiplier
            point_y = map_center[1] - self.robot_position[0].x * self.map_multiplier

            pygame.draw.circle(self.display, (0,0,255), (point_x, point_y), 10.0)
        
        if (self.robot_position[1] != None):
            point_x = map_center[0] - self.robot_position[1].y * self.map_multiplier
            point_y = map_center[1] - self.robot_position[1].x * self.map_multiplier

            pygame.draw.circle(self.display, (0,128,128), (point_x, point_y), 10.0)

    def load_map_from_json(self):
        data = []
        with open(self.map_path) as f:
            for line in f:
                data.append(json.loads(line))
                
        low_x = 0
        low_y = 0
        high_x = 0
        high_y = 0

        for i in range(len(data)):
            map_point = MapPoint()
            map_point.from_dict(data[i])
            if map_point.istag:
                self.map_tag_list.append(map_point)
            else:
                self.map_point_list.append(map_point)

            if map_point.x > high_y:
                high_y = map_point.x
            if map_point.x < low_y:
                low_y = map_point.x
            
            if -map_point.y > high_x:
                high_x = -map_point.y
            if -map_point.y < low_x:
                low_x = -map_point.y
        
        self.map_boundary = [high_y, low_y, -low_x, -high_x]
        if high_x - low_x > high_y - low_y:
            self.map_multiplier = self.map_size/(high_x - low_x)
        else:
            self.map_multiplier = self.map_size/(high_y - low_y)

        x_offset = -(low_x + high_x)/2 * self.map_multiplier
        y_offset = (low_y + high_y)/2 * self.map_multiplier
        self.map_center_offset = (x_offset, y_offset)

def main(args=None):
    rclpy.init()
    n = GameNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()