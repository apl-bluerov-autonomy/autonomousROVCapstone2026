import cv2
import glob
import numpy as np
import matplotlib as plt
from typing import NamedTuple
import time
import os

#TODO CAMERA CALIBRATION
#needs chessboard img i think there is one online
#returns matrix coeff, distortion coeff

def captureImages(run, filepath):
    #http://192.168.2.2/vehicle/autopilot
    # can find the video stream url here in case it changes
    if(run == 'y'):
        print("Hit escape to quit image capture")
        capture = cv2.VideoCapture("rtsp://192.168.2.2:8554/video_udp_stream_0")
        img_counter = 0

        while(1):
            ret, frame = capture.read()
            cv2.imshow('frame', frame)

            k = cv2.waitKey(1)
            if k%256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
            else:
                img_name = "opencv_frame_{}.jpg".format(img_counter)
                imgPath = os.path.join(filepath, img_name)
                print(imgPath)
                cv2.imwrite(imgPath, frame)
                print("{} written!".format(img_name))
                img_counter += 1
                time.sleep(0.2) #capture images at 10 Hz

        capture.release()

        cv2.destroyAllWindows()
    else:
        return


def doCalibration(filepath):
    print("Starting calibration!")
    ARUCO_DICT = cv2.aruco.DICT_4X4_250  # Dictionary ID
    SQUARES_VERTICALLY = 8              # Number of squares vertically
    SQUARES_HORIZONTALLY = 11            # Number of squares horizontally
    SQUARE_LENGTH = 45                   # Square side length (in mm)
    MARKER_LENGTH = 35                   # ArUco marker side length (in mm)
    MARGIN_PX = 5                       # Margins size (in mm)

    dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

    #Create our board
    board = cv2.aruco.CharucoBoard(size=
                                (SQUARES_HORIZONTALLY, SQUARES_VERTICALLY), 
                                squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=dict)
    
    #Detector object
    detector = cv2.aruco.CharucoDetector(board=board)

    #Python lists for our image data
    totalObjPnts = []
    totalImgPnts = []

    # ret, mtx, dist, rvecs, tvecs = 0

    images = glob.glob(filepath)

    #Extract image data
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        results = detector.detectBoard(gray) #results 0 is detected corners, results[1] is markerIDs 

        corners = results[0]; markerIds = results[1]
        if(corners is not None and markerIds is not None):
            object_points, image_points = board.matchImagePoints(corners, markerIds)
            if (len(object_points) >= 10 and len(image_points) == len(object_points)):
                # print("true", fname)
                object_points = object_points.reshape(-1, 3).astype(np.float32) #convert to 2d arr
                image_points  = image_points.reshape(-1, 2).astype(np.float32)
                totalObjPnts.append(object_points)
                totalImgPnts.append(image_points)

    h, w = gray.shape
    image_size = (int(w), int(h))   # width, height openCV needs width height and gray shape is h,w

    if(len(totalImgPnts) == len(totalObjPnts) and len(totalImgPnts) != 0):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                totalObjPnts,
                totalImgPnts,
                image_size,
                None,
                None
            )
    else:
        print(totalImgPnts.size)
        print(totalObjPnts.size)
        print("sizes mismatch")
    return ret, mtx, dist, rvecs, tvecs
    


def main():
    print("Running Camera Calibration:")
    print("Please provide an absolute filepath")
    filepath = input()
    print("Capture images as well as calibrate? (y/n)")
    userIn = input()
    captureImages(userIn, filepath)
    time.sleep(1)
    ret, mtx, dist, rvecs, tvecs = doCalibration(f"{filepath}\\*")
    print(f"Calibration complete:\n ret{ret},\n camera matrix: {mtx},\n distortion coeffs: {dist}")#\n rotation and translation vecs {rvecs}, {tvecs}")
    np.savez('camera_calibration.npz', mtx=mtx, dist=dist) 
    return ret, mtx, dist, rvecs, tvecs


#Actually run the script
main()