import cv2 as cv
import numpy as np

from camera_librealsense import Camera

camera = Camera()
camera.start()
offset = np.array([0.8, 0.75, 0.4])  # robot base relative to the origin in simulator
origin_offset = np.array([0.29, 0.22])  # Offset of Reference Marker to Dynamic Marker
obst_rel_robot = np.array(
    [[0.5, -0.15, 0.05], [0.5, 0.05, 0.05]])  # middle pose relative to robot base


def get_obs_pose(dists):
    # get obstacle pose
    dyn_obstacles = []
    for i in range(len(obst_rel_robot)):
        dyn_obstacle = np.array(obst_rel_robot[i] + offset)
        dyn_obstacles.append(dyn_obstacle)
    for i in range(len(dists)):
        dyn_obstacles[i][0] += dists[i]  # - self.pos_dif
    return np.array(dyn_obstacles)


def add_obs_pose(poses, centers, frame):
    for i in range(len(poses)):
        cv.putText(frame, str(poses[i]),
                   (centers[i * 2][0] - 50, centers[i * 2][1] - 50),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5,
                   (0, 0, 255), 2)
    return frame


while True:
    frame = camera.get_frame()
    distance, frame = camera.get_distance(frame, add_to_frame=True)
    centers, frame = camera.get_centers(frame, add_to_frame=True)
    distance -= origin_offset  # relative to origin
    dyn_obstacles = get_obs_pose(distance)
    # print(centers)
    # print(dyn_obstacles)
    frame = add_obs_pose(dyn_obstacles, centers, frame)
    cv.imshow('frame', frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
camera.stop()
