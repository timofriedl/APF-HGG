import time

import cv2 as cv
import numpy as np
import pyrealsense2 as rs


class Camera:
    def __init__(self):
        # for librealsense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # for aruco detection
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.corners = []
        self.ids = []

    def start(self):
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        time.sleep(2)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_distance(self, frame, add_to_frame=True):
        centers, frame = self.get_centers(frame, add_to_frame=True)
        real_distances = []
        # get idx
        if self.ids is not None and np.all(np.in1d([2, 3, 4, 5], self.ids)):
            ids = list(self.ids)
            bottom_stat_idx = ids.index(4)
            bottom_dyn_idx = ids.index(5)
            top_stat_idx = ids.index(2)
            top_dyn_idx = ids.index(3)

            dist_bottom = np.linalg.norm(centers[bottom_stat_idx] - centers[bottom_dyn_idx], ord=2)
            # dist_bottom = np.abs(centers[bottom_stat_idx][0] - centers[bottom_dyn_idx][0])
            dist_top = np.linalg.norm(centers[top_stat_idx] - centers[top_dyn_idx], ord=2)
            # dist_top = np.abs(centers[top_stat_idx][0] - centers[top_dyn_idx][0])
            dists = [dist_bottom, dist_top]
            indexs = [bottom_stat_idx, top_stat_idx]

            for i in range(len(indexs)):
                corner = self.corners[indexs[i]].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                pixel_len = (abs(topLeft[0] - topRight[0]) + abs(bottomLeft[0] - bottomRight[0])) / 2
                length_per_pixel = 0.042 / pixel_len
                real_distances.append(np.round(dists[i] * length_per_pixel, 3))

            if add_to_frame:
                ids = [bottom_stat_idx, bottom_dyn_idx, top_stat_idx, top_dyn_idx]
                for i in range(len(ids) // 2):
                    cv.line(frame, centers[ids[2 * i]], centers[ids[2 * i + 1]], (255, 0, 0), 2)
                    cv.putText(frame, str(real_distances[i]),
                               (int((centers[2 * i][0] + centers[2 * i + 1][0]) / 2), centers[2 * i][1] - 15),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5,
                               (0, 0, 255), 2)
        return np.array(real_distances), frame

    def get_centers(self, frame, add_to_frame=True):
        (corners, ids, rejected) = self.detector.detectMarkers(frame)
        centers = []
        if ids is not None:  # We have detected some markers
            ids = ids.flatten()
            if self.ids:  # ID's are not empty
                for i in range(len(ids)):  # Iterate over IDs and update the detected ones
                    if ids[i] in self.ids:  # Update Value
                        idx = self.ids.index(ids[i])
                        self.corners[idx] = corners[i]
                    else:  # Append Value
                        self.ids.append(ids[i])
                        self.corners.append(corners[i])
                    pass
            else:  # self.ids is empty
                self.ids = list(ids)
                self.corners = list(corners)
        if self.ids is not None:
            # assert len(self.ids) >= 4, "Not all Arucos detected!!!"
            for i in range(len(self.corners)):
                corner = self.corners[i].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                if add_to_frame:
                    # draw the bounding box of the ArUCo detection
                    cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                if add_to_frame:
                    cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                centers.append([cX, cY])
                # centers[self.ids[i]] = np.array([cX, cY])
        return np.array(centers), frame

    def stop(self):
        self.pipeline.stop()


if __name__ == "__main__":
    camera = Camera()
    camera.start()
    frame = camera.get_frame()

    print("Got Frame, Finishing")
    camera.stop()
