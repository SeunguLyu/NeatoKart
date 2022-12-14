import cv2
import dt_apriltags as apriltag
import numpy as np
import math
import json

class MapPoint():
    '''
    Convenient class to save a Pose in 2D frame and convert that into different forms
    such as transformation matrix and dictionary.
    '''
    def __init__(self, x=0.0, y=0.0, theta=0.0, istag = False, tagid = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.istag = istag
        self.tagid = tagid

    def __str__(self):
        return "x: {}, y: {}. theta: {}".format(self.x, self.y, self.theta)

    def as_matrix(self):
        trans = np.matrix([[math.cos(self.theta), -math.sin(self.theta), self.x],[math.sin(self.theta), math.cos(self.theta), self.y],[0,0,1]])
        return trans

    def from_matrix(self, mat):
        self.theta = math.atan2(mat[1,0], mat[0,0])
        self.x = mat[0,2]
        self.y = mat[1,2]
    
    def get_distance(self):
        return math.sqrt(self.x**2 + self.y**2)

    def to_dict(self):
        dict = {}
        dict["x"] = self.x
        dict["y"] = self.y
        dict["theta"] = self.theta
        dict["istag"] = self.istag
        dict["tagid"] = self.tagid

        return dict
    
    def from_dict(self, dict):
        self.x = dict["x"]
        self.y = dict["y"]
        self.theta = dict["theta"]
        self.istag = dict["istag"]
        self.tagid = dict["tagid"]


def draw_apriltag(detected_image, r):
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

    return detected_image

def get_tag_2d_pose(r):
    ''' This function gets the April Tag's position in camera frame as the input
        and convert that into april's position in the Neato's base frame. '''
    # tag to camera transformation matrix
    t_tag_cam = np.matrix([[r.pose_R[0][0], r.pose_R[0][1], r.pose_R[0][2], r.pose_t[0][0]],
                                [r.pose_R[1][0], r.pose_R[1][1], r.pose_R[1][2], r.pose_t[1][0]],
                                [r.pose_R[2][0], r.pose_R[2][1], r.pose_R[2][2], r.pose_t[2][0]],
                                [0, 0, 0, 1]])
    
    # camera to base transformation matrix
    t_cam_base = np.matrix([[0, 0, 1, 0.2],
                                    [-1, 0, 0, 0],
                                    [0, -1, 0, 0.05],
                                    [0, 0, 0, 1]])

    # tag to base transformation matrix
    t_tag_base = np.dot(t_cam_base, t_tag_cam)
    
    # compute yaw from the tf matrix
    yaw = np.arctan2(t_tag_base[1,2], t_tag_base[0,2])

    pose_to_base = MapPoint(t_tag_base[0,3], t_tag_base[1,3], yaw, True)

    return pose_to_base