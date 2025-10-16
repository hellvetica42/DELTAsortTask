import random
import cv2
import numpy as np

class CameraMapping:

    # Input: Coordinates in map x,y, Output: Coordinates in image u,v [0,1]
    def apply_homography(source_points, H):
        destination_points = []
        for p in source_points:
            x, y, w = H @ np.array([p[0], p[1], 1]).T
            destination_points.append([x/w, y/w])
        return np.array(destination_points)

    def apply_vector_homography(source_points, H):
        destination_points = []
        for p in source_points:
            x, y, w = H @ np.array([p[0], p[1], 0]).T
            destination_points.append([-x, -y])
        return np.array(destination_points)

    # get assumed default intrinsics of camera
    def get_homography_matrix(source_points, destination_points):
        image_to_map, status = cv2.findHomography(source_points, destination_points)
        return image_to_map, status

    def map_from_pixels_to_uv(px_points, frame_size):
        uv_points = px_points.copy().astype(np.float32)
        uv_points[:, 0] /= frame_size[0]
        uv_points[:, 1] = 1 - (uv_points[:, 1] / frame_size[1])
        return uv_points

    def map_from_uv_to_pixels(uv_points, frame_size):
        px_points = uv_points.copy()
        px_points[:, 0] *= frame_size[0]
        px_points[:, 1] = uv_points[:, 1] * frame_size[1]
        return px_points

    def get_calibrated_height(point_mm):
        points = np.array([
            [-150, 60],
            [100, 300],
            [350, -100],
            [150, -250],
            [0,0],
            [350, 0],
            [0, 200],
            [0, 350],
            [200, 100]
        ])

        heights = np.array([
            -630,
            -625,
            -610,
            -615,
            -625,
            -611,
            -628,
            -628,
            -618
        ])
        distances = np.linalg.norm(points - point_mm, axis=1)
        if np.any(distances == 0):
        # Return the value of the matching point
            return heights[np.argmin(distances)]

        weights = 1 / distances

        weights = weights / np.sum(weights)

        height = weights @ heights
        
        return height
