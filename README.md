# NeatoKart

![](documents/images/neatokart_title_screen.gif)

## Introduction

Inspired by Mario Kart Live: Home Circuit (2020), this project aimed to build a Neato (vacuum robot)-racing
experience that involves multiple Neatos at the same time, with users’ ability to create their
own circuits, use items during the race, and a UI that users can look at while playing the
game.

## Demo

[![Introduction Video](https://img.youtube.com/vi/lLSGq1WrFXU/maxresdefault.jpg)](https://youtu.be/lLSGq1WrFXU)
↑ Click to view the video

[![Play Demo](documents/images/race_demo_thumbnail.gif)](https://youtu.be/6Exe3-IEtOo)
↑ Click to view the video

## Run
1. Follow steps in this [page](https://comprobo22.github.io/How%20to/setup_your_environment) to set up the environment
2. Clone [neato_packages](https://github.com/SeunguLyu/neato_packages.git) in the ros2 src folder
3. Clone this repo in the ros2 src folder and build
4. Run the command to install the necessary Python packages
```
pip install pygame
pip install dt-apriltags
```
### Simulation
1. Run the following command to start the gazebo
```
ros2 launch neato2_gazebo neato_maze.py
```
2. Run the create_map node to save a map. This step can be skipped if the map already exists. For further instruction, look at the "Creating Map" section
```
ros2 run neato_kart create_map
```
3. Change self.map_name in drive_neato.py to the name of the saved map. Run the drive_neato node with the following command
```
ros2 run neato_kart drive_neato
```
4. While the drive_neato node is running, run the game_node
```
ros2 run neato_kart game_node
```

### Real World
For the real-world setup, this has been tested in a specific network environment (Olin College MAC) where users can remotely connect to each Neatos that runs their nodes through the network. For further information on how this environment is set up, go to this [website](https://comprobo22.github.io/) and look at the "Teaching Team Documentation" section.

1. Switch to the 'multiagent_support' branch in the 'neato_packages' repo, after fetching the upstream repository with the following command:
```
git fetch upstream
```
2. Connect to first Neato with the following command, with the correct IP
```
$ ros2 launch neato_node2 bringup_multi.py host:=neato1-ip-address-here robot_name:=robot1 udp_video_port:=5002 udp_sensor_port:=7777 gscam_config:='udpsrc port=5002 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```
3. Connect to the second Neato with the following command, with the correct IP
```
ros2 launch neato_node2 bringup_multi.py host:=neato2-ip-address-here robot_name:=robot2 udp_video_port:=5003 udp_sensor_port:=7778 gscam_config:='udpsrc port=5003 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```
4. Use the first Neato to create a map with the following command
```
ros2 run neato_kart create_map --ros-args -p robot_name:="robot1"
```
5. Change self.map_name in drive_neato.py to the name of the saved map. Run two separate drive_neato nodes with the following commands:
```
ros2 run neato_kart drive_neato --ros-args -p robot_name:="robot1"
```
```
ros2 run neato_kart drive_neato --ros-args -p robot_name:="robot2"
```
6. While drive_neato nodes are running, run the game_node
```
ros2 run neato_kart game_node --ros-args -p robot1_name:="robot1" -p robot2_name:="robot2"
```

## Control
1. Global
```
SPACE: Start the Game
```
1. Player 1
```
W: Accelerate
S: Decelerate, Use Item Backward
A: Steer Left
D: Steer Right
L_SHIFT: Use Item
```
2. Player 2
```
UP: Accelerate
DOWN: Decelerate, Use Item Backward
LEFT: Steer Left
RIGHT: Steer Right
R_SHIFT: Use Item
```
## Guide
### Creating Checkpoints

![](documents/images/checkpoint.png)

Checkpoints can be in any dimension as far as Neato can pass under them. The only requirement is that each checkpoint should have one unique [36h11 AprilTag](https://docs.cbteeple.com/assets/files/tag36h11_100mm.pdf?fbclid=IwAR0peNEudVlSOC_sq_ZHKFJyY8_Y2BiRSEgcBE1R_nNtqmpJo2MMvEQc5Hc) attached. Preferably, the size should be around 5~15cm for better detection. Once the size is determined, the tag size in both drive_neato.py and create_map.py run_loop function should be changed to the corresponding size:
```
results = self.detector.detect(gray, estimate_tag_pose = True, camera_params=self.camera_param, tag_size = YOUR_TAG_SIZE)
```
For convenience, we made a [3D stand model](src/NeatoKart/documents/checkpoint_stand.stl) to hold cardboard boxes. There's no limit on how many checkpoints can be used for this program, but every tag should be unique, or else the game will not work properly.

### Creating Map

[![Creating Map](https://img.youtube.com/vi/I1t8w_XXLrw/maxresdefault.jpg)](https://youtu.be/I1t8w_XXLrw)
↑ Click to view the video

The above video shows how a new map is created. It is pretty simple - run create_map node, click the OpenCV screen, drive a Neato through checkpoints, and click the OpenCV screen again to save. 

![](documents/images/minimap_capture.png)

[Saved File From This Drive](maps/example_map.json)

The resulting map looks like the above minimap. The green points indicate checkpoints, and two big points (blue and cyan) represent Neato's position on the map.

### Start Race

![](documents/images/before_start.png)
To start the game, there are required conditions to not have errors:

1. Both drive_neato nodes should be running with the correct robot name parameter
2. Both Neatos should have detected the first April Tag and see the track on the screen
3. When Neato move, the camera feed is updated in real-time

If all these conditions are met, you are safe to press the SPACE button and start the game! It is important to pass checkpoints in the order that the map was recorded, or else the game will not be able to detect the end condition. Once a player goes through all the checkpoints by order, the game ends.

## Structure
### Map

```json
{"x": 0.5264, "y": 0.0022, "theta": 0.0040, "istag": false, "tagid": 0}
```
Every point/tag will be saved in the above JSON format and will be loaded as a MapPoint object.

```python
class MapPoint():
    '''
    Convenient class to save a Pose in the 2D frame and convert that into different forms such as transformation matrix and dictionary.
    '''
    def __init__(self, x=0.0, y=0.0, theta=0.0, istag = False, tagid = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.istag = istag
        self.tagid = tagid
```
MapPoint object is a class that helps convert a 2D pose to a transformation matrix or dictionary so that it can be used by multiple functions. This object is used as a base for every frame transformation in this project.

The map's origin is set as (x:0, y:0, theta:0) and every point/tag in the map is saved relative to this origin. Later when the map is loaded by the Neato and in its Odom frame, the map's origin will be updated in the Odom frame in real time based on the detected tag position so that the whole map can be updated at the same time.

### Items
1. Banana

![](documents/images/banana.gif)
![](documents/images/banana2.gif)

The banana will stay in the map's position where the user first used the item. Any user that steps on it will be rotating at the same position for a certain amount of seconds.

2. Turtle Shell

![](documents/images/turtle.gif)
![](documents/images/turtle2.gif)

The turtle shell will move in the direction of the angle at which it was first created. Any user that is hit by the turtle will be rotating at the same position for a certain amount of seconds.

3. Boost

![](documents/images/boost.gif)

Boost will raise the Neato's speed by 20% for 5 seconds. 

### Track

1. How to track is created and drawn

### Minimap

![](documents/images/minimap_capture.png)

Minimap is a tool to check the condition of what is happening in the game and the current map. The minimap will draw the whole map on a small scale, and show the locations of the checkpoints, items, and players. The minimap is updated in real-time based on the pose data of each Neatos in the Map frame.

Minimap is created based on the MapPoints loaded by a saved JSON file. One important aspect is resizing the map so that it can fit into the size of the minimap. The code to resize the minimap based on its shape is written [here](https://github.com/SeunguLyu/NeatoKart/blob/cd0da6d8b62255f77d26d52316ad13e4a6b0ccaa/neato_kart/game_node.py#L651), where map_multiplier is set so that longer side (x or y) will be the same as the map size. For example, if the x side was 10m and y side was 5m and the minimap size is 200 pixels, the map multiplier will be 20 so that minimap does not exceed the boundaries (then the x side will be 200 pixels, the y side is 100 pixels, thus does not exceed the minimap boundaries)

### AprilTag and Checkpoints

![](documents/images/checkpoint_demo.gif)

Checkpoints are an essential feature of the game to check the progress of each Neatos. Checkpoints have attached April Tag with a unique tag id, which is mainly used for robot localization. We also implemented a way to detect Neato passing the checkpoint to figure out the race progress and give items to the user. Checkpoints serve several different purposes:

1. Update the Map frame position in the Odom frame. Because errors can happen easily between the real position of the Neato and the Odom frame, there must be a way to calibrate these errors. Having multiple checkpoints help the user to localize themself in the Map frame due to frequent updating.
2. By checking that the user is passing by checkpoints in order, we can define the winning condition for this game as "The first user who passed all the checkpoints and come back to the first checkpoint"
3. To ensure that there is still a chance for the losing player to win, items are given to each user every time they pass by the checkpoint. 

## System architecture

### Camera Matrix

### Frame Transformation
1. Camera frame to base frame (April Tag)
2. Base frame to Odom frame (Neato Position), April tag pose in Odom.
3. april tag pose in map frame, april tag pose in odom frame, map frame pose in odom frame
4. map frame odom, base to odom, we can know neato position in map frame. (Game)
5. neato pose in odom frame, map pose in odom frame, map pose in neato base frame (draw Track)

### ROS Nodes
1. Drive Neato (topics processed images, neato position) subs pubs
2. Game Node receives the info subs pubs diagram

### ROS and PyGame
1. Why Pygame, key input, UI

## Project Stories
[Project Story 1](documents/project_story_1.pdf)

[Project Story 2](documents/project_story_2.pdf)

