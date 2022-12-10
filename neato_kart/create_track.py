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
        
        self.d = 1
        self.l = 1
        self.waypoints = []
        self.outertrack = []
        self.innertrack = []
        self.pnum = 0

    def generate_track_waypoint(self, waypoint):
        x = waypoint.x
        y = waypoint.y
        theta = waypoint.theta

        d1 = [0, 0.2, 1]
        d2 = [0, -0.2, 1]
        
        outer = np.dot(waypoint.as_matrix(), d1)
        inner = np.dot(waypoint.as_matrix(), d2)

        outer = TrackPoint(outer[0, 0], outer[0, 1], 0, 0)
        inner = TrackPoint(inner[0, 0], inner[0, 1], 0, 0)

        self.outertrack.extend([outer])
        self.innertrack.extend([inner])


    def generate_track(self, d, l, waypoints):
        self.d = d
        self.l = l
        self.outertrack = []
        self.innertrack = []
        self.waypoints = waypoints
        for waypoint in self.waypoints:
            self.generate_track_waypoint(waypoint)
        
        return self.outertrack + self.innertrack

    def get_track_waypoints(self):
        return self.outertrack + self.innertrack
    
    def draw_track(self, waypoints, image):
        self.waypoints = waypoints
        self.generate_track(1, 1, waypoints)
        
        track_image = image

        # transformation matrix from base link to camera frame
        t_cam_base = np.matrix([[0, 0, 1, 0.2],
                                [-1, 0, 0, 0],
                                [0, -1, 0, 0.15],
                                [0, 0, 0, 1]])

        # camera intrinsics
        #K = np.matrix([[971.646825, 0.000000, 501.846472], 
        #               [0.000000, 972.962863, 402.829241], 
        #               [0.000000, 0.000000, 1.000000]])


        K = np.matrix([[971.646825, 0.000000, 1024/2], 
                       [0.000000, 972.962863, 768/2], 
                       [0.000000, 0.000000, 1.000000]])
        # calculate pixel coords from base link coords
        pixel_coords = []
        w = []

        if len(self.waypoints) != 0:
            first_point = (int(self.waypoints[0].x), int(self.waypoints[0].y))

            for waypoint in self.waypoints:
                waypoint = [waypoint.x, waypoint.y, 0, 1]
                print("waypoint " + str(waypoint))

                w.append(waypoint)

                cam_waypoint = np.dot(np.linalg.inv(t_cam_base), waypoint)
                cam_waypoint = np.delete(cam_waypoint, -1)
                print("camwaypoint " + str(cam_waypoint))
                pixel_coord_unnorm = np.dot(K, cam_waypoint.T)
                if pixel_coord_unnorm[-1] < 0:
                    continue
                pixel_coord_norm = np.divide(pixel_coord_unnorm, pixel_coord_unnorm[-1]) 

                # store all the pixels
                pixel_coords.append(pixel_coord_norm)

                # print cx, cy to check the opencv dim
                cx = track_image.shape[0]
                cy = track_image.shape[1]
                px = pixel_coord_norm[0]
                py = pixel_coord_norm[1]
                print("pixels: ", px, py)

                # Center coordinates
                center_coordinates = (int(px), int(py))
                
                # Radius of circle
                radius = 3
                
                # Blue color in BGR
                color = (255, 0, 0)
                
                # Line thickness of 2 px
                thickness = 2
                
                # Using cv2.circle() method
                # Draw a circle with blue line borders of thickness of 2 px
                track_image = cv2.circle(track_image, center_coordinates, radius, color, thickness) 
                # track_image = cv2.putText(track_image, center_coordinates, center_coordinates, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), thickness, cv2.LINE_AA)
                track_image = cv2.line(track_image, first_point, center_coordinates, color, 5)
                
                first_point = center_coordinates

        if len(self.outertrack) != 0:
            self.track_points = self.outertrack
            first_point = (int(self.track_points[0].x), int(self.track_points[0].y))

            for waypoint in self.track_points:
                waypoint = [waypoint.x, waypoint.y, 0, 1]

                w.append(waypoint)

                cam_waypoint = np.dot(np.linalg.inv(t_cam_base), waypoint)
                cam_waypoint = np.delete(cam_waypoint, -1)
                pixel_coord_unnorm = np.dot(K, cam_waypoint.T)

                if pixel_coord_unnorm[-1] < 0:
                    continue
                pixel_coord_norm = np.divide(pixel_coord_unnorm, pixel_coord_unnorm[-1]) 

                # store all the pixels
                pixel_coords.append(pixel_coord_norm)

                # print cx, cy to check the opencv dim
                cx = track_image.shape[0]
                cy = track_image.shape[1]
                px = pixel_coord_norm[0]
                py = pixel_coord_norm[1]
                print("pixels: ", px, py)

                # Center coordinates
                center_coordinates = (int(px), int(py))
                
                # Radius of circle
                radius = 3
                
                # Blue color in BGR
                color = (0, 255, 0)
                
                # Line thickness of 2 px
                thickness = 2
                
                # Using cv2.circle() method
                # Draw a circle with blue line borders of thickness of 2 px
                # track_image = cv2.circle(track_image, center_coordinates, radius, color, thickness) 
                # track_image = cv2.putText(track_image, center_coordinates, center_coordinates, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), thickness, cv2.LINE_AA)
                track_image = cv2.line(track_image, first_point, center_coordinates, color, 5)
                
                first_point = center_coordinates

        if len(self.innertrack) != 0:
            self.track_points = self.innertrack
            first_point = (int(self.track_points[0].x), int(self.track_points[0].y))

            for waypoint in self.track_points:
                waypoint = [waypoint.x, waypoint.y, 0, 1]
                print("waypoint " + str(waypoint))

                w.append(waypoint)

                cam_waypoint = np.dot(np.linalg.inv(t_cam_base), waypoint)
                cam_waypoint = np.delete(cam_waypoint, -1)
                print("camwaypoint " + str(cam_waypoint))
                pixel_coord_unnorm = np.dot(K, cam_waypoint.T)
                if pixel_coord_unnorm[-1] < 0:
                    continue
                pixel_coord_norm = np.divide(pixel_coord_unnorm, pixel_coord_unnorm[-1]) 

                # store all the pixels
                pixel_coords.append(pixel_coord_norm)

                # print cx, cy to check the opencv dim
                cx = track_image.shape[0]
                cy = track_image.shape[1]
                px = pixel_coord_norm[0]
                py = pixel_coord_norm[1]
                print("pixels: ", px, py)

                # Center coordinates
                center_coordinates = (int(px), int(py))
                
                # Radius of circle
                radius = 3
                
                # Blue color in BGR
                color = (0, 255, 0)
                
                # Line thickness of 2 px
                thickness = 2
                
                # Using cv2.circle() method
                # Draw a circle with blue line borders of thickness of 2 px
                # track_image = cv2.circle(track_image, center_coordinates, radius, color, thickness) 
                # track_image = cv2.putText(track_image, center_coordinates, center_coordinates, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), thickness, cv2.LINE_AA)
                track_image = cv2.line(track_image, first_point, center_coordinates, color, 5)
                
                first_point = center_coordinates


        # print("W: ", w)
        return track_image



import matplotlib.pyplot as plt


def build_cartesian_plane():
    # Enter x and y coordinates of points and colors
    xs = [0.5480779617886374, 1.0481041973901764, 1.0481041973901764]
    ys = [0.00043349163114081435, 0.0008407295545138638, 0.001285098641959248]
    colors = ['m', 'g', 'r']
    
    P1 = MapPoint(0.5480779617886374, 0.00043349163114081435, 0.0008140771480200654)
    P2 = MapPoint(1.0481041973901764, 0.0008407295545138638, 0.0008117805123801778)
    P3 = MapPoint(1.5980900496895702, 0.001285098641959248, 0.0008079034145749291)
    waypoints = [P1, P2, P3]
    track = CreateTrack()
    track.generate_track(1, 2, waypoints)
    
    xxs = []
    yys = []

    for i in range(len(track.outertrack)):
        xxs.append(track.outertrack[i].x)
        xxs.append(track.innertrack[i].x)
        yys.append(track.outertrack[i].y)
        yys.append(track.innertrack[i].y)

    print("outer: ", xxs)
    print("inner: ", yys)

    # Select length of axes and the space between tick labels
    xmin, xmax, ymin, ymax = -6, 6, -6, 6
    ticks_frequency = 1

    # Plot points
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.scatter(xs, ys, c=colors)
    # colorss = ['m', 'm', 'm', 'm', 'g', 'g', 'g', 'g', 'r', 'r', 'r', 'r']
    colorss = ['m', 'm', 'g', 'g', 'r', 'r']
    ax.scatter(xxs, yys, c=colorss)

    # Draw lines connecting points to axes
    for x, y, c in zip(xs, ys, colors):
        ax.plot([x, x], [0, y], c=c, ls='--', lw=1.5, alpha=0.5)
        ax.plot([0, x], [y, y], c=c, ls='--', lw=1.5, alpha=0.5)

    # Set identical scales for both axes
    ax.set(xlim=(xmin-1, xmax+1), ylim=(ymin-1, ymax+1), aspect='equal')

    # Set bottom and left spines as x and y axes of coordinate system
    ax.spines['bottom'].set_position('zero')
    ax.spines['left'].set_position('zero')

    # Remove top and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Create 'x' and 'y' labels placed at the end of the axes
    ax.set_xlabel('x', size=14, labelpad=-24, x=1.03)
    ax.set_ylabel('y', size=14, labelpad=-21, y=1.02, rotation=0)

    # Create custom major ticks to determine position of tick labels
    x_ticks = np.arange(xmin, xmax+1, ticks_frequency)
    y_ticks = np.arange(ymin, ymax+1, ticks_frequency)
    ax.set_xticks(x_ticks[x_ticks != 0])
    ax.set_yticks(y_ticks[y_ticks != 0])

    # Create minor ticks placed at each integer to enable drawing of minor grid
    # lines: note that this has no effect in this example with ticks_frequency=1
    ax.set_xticks(np.arange(xmin, xmax+1), minor=True)
    ax.set_yticks(np.arange(ymin, ymax+1), minor=True)

    # Draw major and minor grid lines
    ax.grid(which='both', color='grey', linewidth=1, linestyle='-', alpha=0.2)

    # Draw arrows
    arrow_fmt = dict(markersize=4, color='black', clip_on=False)
    ax.plot((1), (0), marker='>', transform=ax.get_yaxis_transform(), **arrow_fmt)
    ax.plot((0), (1), marker='^', transform=ax.get_xaxis_transform(), **arrow_fmt)

    plt.show()


# build_cartesian_plane()
