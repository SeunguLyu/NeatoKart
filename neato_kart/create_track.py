import cv2
import math
import numpy as np
from neato_kart.detect_april_tag import MapPoint

class TrackPoint():
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
        # super().__init__('create_track')
        
        self.d = 2
        self.l = 1
        self.waypoints = []
        self.outertrack = []
        self.innertrack = []
        self.pnum = 0

    def generate_track_waypoint(self, waypoint):
        '''
        Generates a pair of track outerpoint and track innerpoint for every waypoint
        that is received.
        '''

        # create matrix for specifying outer/inner track point location from center point
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
        the waypoints
        '''
        self.d = d              # d = (width of track)/2 
        self.outertrack = []    # initialize every draw because the waypoints change
        self.innertrack = []    # same as above
        self.waypoints = waypoints

        # generate track outerpoint and innerpoint for every waypoint 
        for waypoint in self.waypoints:
            self.generate_track_waypoint(waypoint)
        
        return self.outertrack + self.innertrack

    
    def draw_track(self, waypoints, image):
        track_image = image
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


        # self.K = np.matrix([[971.646825, 0.000000, 1024/2], 
        #                [0.000000, 972.962863, 768/2], 
        #                [0.000000, 0.000000, 1.000000]])

        center_track = self.get_track_pixel_points(self.waypoints)
        out_track = self.get_track_pixel_points(self.outertrack)
        in_track = self.get_track_pixel_points(self.innertrack)
                      
        center_track = np.asarray(center_track)
        out_track = np.asarray(out_track)
        in_track = np.asarray(in_track)

        track_image = cv2.polylines(track_image, [center_track], True, (255, 0, 0), 5)
        track_image = cv2.polylines(track_image, [out_track], True, (0, 255, 0), 5)
        track_image = cv2.polylines(track_image, [in_track], True, (0, 255, 0), 5)
        return track_image

    def get_track_pixel_points(self, track):
        processed_track = []
        if len(track) != 0:
            for waypoint in track:
                waypoint = [waypoint.x, waypoint.y, 0, 1]
                cam_waypoint = np.dot(np.linalg.inv(self.t_cam_base), waypoint)
                cam_waypoint = np.delete(cam_waypoint, -1)
                pixel_coord_unnorm = np.dot(self.K, cam_waypoint.T)

                if pixel_coord_unnorm[-1] < 0:
                    continue
                pixel_coord_norm = np.divide(pixel_coord_unnorm, pixel_coord_unnorm[-1]) 

                # get pixel coordinates in camera view
                px = pixel_coord_norm[0]
                py = pixel_coord_norm[1]

                processed_track.append((int(px), int(py)))
        
        return processed_track
    