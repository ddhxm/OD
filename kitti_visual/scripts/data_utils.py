import cv2
import pandas as pd
import numpy as np

IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 
                    'ax', 'ay', 'az', 'af', 'al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
                    'posacc', 'velacc', 'navstat', 'numsats', 'posmode', 'velmode', 'orimode']

TRACKING_COLUMNS_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top', 
                          'bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']


def read_camera(path):
    return cv2.imread(path)

def read_point_cloud(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)

def read_imu(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df

def read_tracking(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = TRACKING_COLUMNS_NAMES
    df.loc[df.type.isin(['Truck', 'Van', 'Tram']), 'type'] = 'Car'
    df = df[df.type.isin(['Car', 'Pedestrian', 'Cyclist'])]
    return df

