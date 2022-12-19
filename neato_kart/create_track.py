import cv2
import math
import numpy as np
from neato_kart.detect_april_tag import MapPoint

class TrackPoint():
    '''
    Convenient class to save a Pose in 2D frame and convert that into different forms
    such as transformation matrix.
    '''

    def __init__(self, x=0.0, y=0.0, theta=0.0, pnum=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.pnum = pnum
    
    def as_matrix(self):
        trans = np.matrix([[math.cos(self.theta), -math.sin(self.theta), self.x],[math.sin(self.theta), math.cos(self.theta), self.y],[0,0,1]])
        return trans

    def from_matrix(self, mat):
        self.theta = math.atan2(mat[1,0], mat[0,0])
        self.x = mat[0,2]
        self.y = mat[1,2]


class CreateTrack():
    def __init__(self):        
        self.d = 0.1
        self.waypoints = []
        self.outertrack = []
        self.innertrack = []
        self.pnum = 0

    def generate_track_waypoint(self, waypoint):
        '''
        Generates a pair of track outerpoint and track innerpoint for every waypoint 
        that is received.

        @param waypoint: point that forms teh track in base_link frame
        '''

        # create matrix to specify outer/inner track point location from center point
        d1 = [0, self.d, 1]
        d2 = [0, -self.d, 1]
        
        # since every waypoint is a frame of it's own, can perform simple matrix mult
        outer = np.dot(waypoint.as_matrix(), d1)
        inner = np.dot(waypoint.as_matrix(), d2)

        # points are on the ground, so only extract x, y coords
        outer = TrackPoint(outer[0, 0], outer[0, 1], 0, 0)
        inner = TrackPoint(inner[0, 0], inner[0, 1], 0, 0)

        # update outertrack and innertrack
        self.outertrack.extend([outer])
        self.innertrack.extend([inner])


    def generate_track(self, d, waypoints):
        '''
        Generate all the points that composes the outertrack and the innertrack given
        the waypoints.

        @param d: distance from centertrack to outer/inner track
        @param waypoints: list of points that form the track (excluding tag points) 
                          in base_link frame
        '''
        self.d = d              # d = (width of track)/2 
        self.outertrack = []    # initialize every draw because the waypoints change
        self.innertrack = []    # same as above
        self.waypoints = waypoints

        # generate track outerpoint and innerpoint for every waypoint 
        for waypoint in self.waypoints:
            self.generate_track_waypoint(waypoint)
        

    def draw_track(self, waypoints, image):
        """
        Given an OpenCV image and the track points relevant from the pose that produced
        that image, draws center, outer, and inner tracks on the image.

        @param waypoints: list of points that form the track (excluding tag points) 
                          in base_link frame
        @param image: current OpenCV image 

        @return track_image: OpenCV image with the track (CV lines) drawn
        """

        track_image = image

        # generate outertrack and innertrack points for every image frame/movement
        self.generate_track(0.2, waypoints)

        # transformation matrix from base link to camera frame
        self.t_cam_base = np.matrix([[0, 0, 1, 0.2],
                                     [-1, 0, 0, 0],
                                     [0, -1, 0, 0.15],
                                     [0, 0, 0, 1]])

        # camera intrinsics
        self.K = np.matrix([[971.646825, 0.000000, 501.846472], 
                            [0.000000, 972.962863, 402.829241], 
                            [0.000000, 0.000000, 1.000000]])

        # convert track points in base_link frame to pixel coords
        center_track = self.get_track_pixel_points(self.waypoints)
        out_track = self.get_track_pixel_points(self.outertrack)
        in_track = self.get_track_pixel_points(self.innertrack)

        # draw lines to connect track points for center, outer, inner tracks
        self.draw_continuous_line(track_image, center_track, (255,255,255))
        self.draw_continuous_line(track_image, out_track, (0,255,0))
        self.draw_continuous_line(track_image, in_track, (0,255,0))

        return track_image

    def get_track_pixel_points(self, track):
        """
        Given a list of points that form a track, convert the points into pixel coordinates.

        @param track: list of base_link frame points that form the center/outer/inner track

        @return a list of track points represented as pixel coordinates
        """

        processed_track = []
        if len(track) != 0:
            for waypoint in track:
                waypoint = [waypoint.x, waypoint.y, 0, 1]

                # transform waypoint from base_link to camera frame
                cam_waypoint = np.dot(np.linalg.inv(self.t_cam_base), waypoint)
                cam_waypoint = np.delete(cam_waypoint, -1)

                # transform waypoint from camera frame to pixel coords
                pixel_coord_unnorm = np.dot(self.K, cam_waypoint.T)

                # filter out the waypoints with -z pixel values 
                # (these are points that are behind the neato's field of view)
                if pixel_coord_unnorm[-1] < 0:
                    processed_track.append(None)    # we do this to prevent tracks jumping
                    continue

                # normalize with respect to z-value
                pixel_coord_norm = np.divide(pixel_coord_unnorm, pixel_coord_unnorm[-1]) 

                # get pixel coordinates in camera view
                px = pixel_coord_norm[0]
                py = pixel_coord_norm[1]

                processed_track.append((int(px), int(py)))
        
        return processed_track
    
    def draw_continuous_line(self, image, track, color):
        """
        Draw a track on the image by connecting two points with OpenCV line.

        @param image: OpenCV image to draw the track (CV lines)
        @param track: list of pixel coordinate points that form the center/outer/inner track
        @param color: BGR value to draw the track
        """
        for i in range(len(track)-1):
            if track[i] != None and track[i+1] != None:
                cv2.line(image, track[i], track[i+1], color, 5)
