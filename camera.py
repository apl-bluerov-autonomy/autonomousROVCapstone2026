import math
from pymavlink import mavutil
import time
import numpy as np
import cv2
import os


class camera:
    mtx = 0
    dist = 0
    markerSize = 127 #mm
    capture = None
    streamOk = False


    def __init__(self, markerSize=127):
        self.markerSize = markerSize
        print("Creating camera object")
        pass

    def updateMarkerSize(self, size):
        self.markerSize = size

    def startStream(self):
        try:
            self.capture = cv2.VideoCapture("rtsp://192.168.2.2:8554/video_stream__dev_video2")
            print("Stream found")
            self.streamOk = True
        except:
            self.streamOk = False

    def loadCameraSettings(self):
        print("loading settings")
        calibration_data = np.load('camera_calibration.npz')

        # Access the individual arrays
        self.mtx = calibration_data['mtx']
        self.dist = calibration_data['dist'] 
        return self.mtx, self.dist


    def getImg(self):
        if(self.streamOk):
            ret, frame = self.capture.read()
            return frame
        else:
            return None
    
    def release(self):
        self.capture.release()
        cv2.destroyAllWindows()

    ##Borrowed from stack overflow
    ##https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    def my_estimatePoseSingleMarkers(self, corners, marker_size):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, self.mtx, self.dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash


    def poseEstimate(self, img):
        rvecCollection = np.empty(0)
        tvecCollection = np.empty(0)

        dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        params = cv2.aruco.DetectorParameters()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(dict, params)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            for id in ids:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                rvec, tvec, _ = self.my_estimatePoseSingleMarkers(corners, self.markerSize)
                rvecCollection.append(rvec)
                tvecCollection.append(tvec)
                time.sleep(1e-4) #can get rid of this maybe
            return np.array(rvecCollection), np.array(rvecCollection), ids
        return None

    def rvecToMat(self, rvec):
        matrix, _ = cv2.Rodrigues(rvec)
        return matrix

    def createTransformationMatrix(self, rvec, tvec):
        mat = self.rvecToMat(rvec)
        T = np.eye(4)
        T[:3, :3] = mat
        T[:3, 3] = tvec.flatten()
        return T

    def getPos(self):
        img = self.getImg()
        vecs = self.poseEstimate(img)
        vecs = vecs[0]
        if(vecs is not None):
            rvec, tvec, ids = vecs
            print('Tag found with id:', ids)
            mat = self.createTransformationMatrix(rvec, tvec)
            x = mat[0][3]; y = mat[1][3]; z = mat[2][3]
            rot = mat[0:2][0:2]
            # print(mat)
            return x, y, z, rot, rvec #in mm
        else:
            print("No tag detected")
            return None


