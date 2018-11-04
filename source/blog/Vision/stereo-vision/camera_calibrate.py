#!/usr/bin/env python3
import numpy as np
import cv2
from glob import glob
# import argparse
from enum import Enum
import time
import pickle


Markers = Enum('Markers', 'checkerboard acircle')


class Rectify(object):
    def __init__(self, filename, alpha=0.5):
        self.info = self.__read(filename)
        self.alpha = alpha

    def __read(self, filename, handler=pickle):
        with open(filename, 'rb') as f:
            data = handler.load(f)
        # print(data)
        return data

    def __fix(self, image, m, d):
        """
        image: an image
        alpha = 0: returns undistored image with minimum unwanted pixels (image
                    pixels at corners/edges could be missing)
        alpha = 1: retains all image pixels but there will be black to make up
                    for warped image correction
        """
        h,w = image.shape[:2]
        # Adjust the calibrations matrix
        # alpha=0: returns undistored image with minimum unwanted pixels
        #           (image pixels at corners/edges could be missing)
        # alpha=1: retains all image pixels but there will be black to make
        #           up for warped image correction
        # returns new cal matrix and an ROI to crop out the black edges
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(m, d, (w, h), self.alpha)
        # undistort
        ret = cv2.undistort(image, m, d, None, newcameramtx)
        return ret

    def undistort(self, image):
        mtx = self.info['cameraMatrix']
        dist = self.info['distCoeffs']
        return self.__fix(image, mtx, dist)

    def undistortLeft(self, image):
        mtx = self.info['cameraMatrix1']
        dist = self.info['distCoeffs1']
        return self.__fix(image, mtx, dist)

    def undistortRight(self, image):
        mtx = self.info['cameraMatrix2']
        dist = self.info['distCoeffs2']
        return self.__fix(image, mtx, dist)

    def undistortStereo(self, left, right):
        return self.undistortLeft(left), self.undistortRight(right)


class SCamera(object):
    def imshow(self, imgs, scale=3, msec=500):
        for i, img in enumerate(imgs):
            h,w = img.shape[:2]
            cv2.imshow('image-{}'.format(i), cv2.resize(img, (w//scale,h//scale)))
            cv2.waitKey(msec)

    def get_images(self, path):
        """
        Given a path, it reads all images. This uses glob to grab file names
        and excepts wild cards *
        Ex. cal.getImages('./images/*.jpg')
        """
        imgs_l = []
        imgs_r = []
        files = glob(path)

        print("Found {} images at {}".format(len(tuple(files)), path))
        # print('-'*40)

        for i, f in enumerate(files):
            img = cv2.imread(f, 0)
            if img is None:
                raise Exception('>> Could not read: {}'.format(f))
            else:
                if len(img.shape) > 2:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # print("[{}]:{} ({}, {})".format(i, f, *img.shape))
                h, w = img.shape
                l = img[:, :w//2]
                r = img[:, w//2:]
                imgs_l.append(l)
                imgs_r.append(r)
        # print('-'*40)
        return imgs_l, imgs_r


class CameraCalibration(object):
    def __init__(self):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    def save(self, filename, handler=pickle):
        with open(filename, 'wb') as f:
            handler.dump(self.info, f)

    def calibrate(self, images, marker_type, marker_size):
         """
         images: an array of grayscale images, all assumed to be the same size
         """
         self.marker_type = marker_type
         self.marker_size = marker_size
         self.save_cal_imgs = []

         # Arrays to store object points and image points from all the images.
         objpoints = []  # 3d point in real world space
         imgpoints = []  # 2d points in image plane.

         max_corners = self.marker_size[0]*self.marker_size[1]

         print("Images: {} @ {}".format(len(images), images[0].shape))
         print("{} {}".format(marker_type, marker_size))
         print('-'*40)
         for cnt, gray in enumerate(images):
             orig = gray.copy()
             if len(gray.shape) > 2:
                 gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

             ret, objpoints, imgpoints, corners = self.findMarkers(gray, objpoints, imgpoints)
             # If found, add object points, image points (after refining them)
             if ret:
                 print('[{}] + found {} of {} corners'.format(cnt, corners.size / 2, max_corners))
                 term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001)
                 cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)

                 # Draw the corners
                 tmp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                 cv2.drawChessboardCorners(tmp, self.marker_size, corners, True)
                 self.save_cal_imgs.append(tmp)
             else:
                 print('[{}] - Could not find markers'.format(cnt))

         # h, w = images[0].shape[:2]
         # rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

         # images size here is backwards: w,h
         rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)
         print("RMS error: {}".format(rms))
         print('-'*40)

         self.data = {
             'date': time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()),
             'markerType': marker_type,
             'markerSize': marker_size,
             'imageSize': imgs_l[0].shape,
             'cameraMatrix': mtx,
             'distCoeffs': dist,
             'rms': rms,
             'rvecs': rvecs,
             'tvecs': tvecs
        }
         # self.data = {'camera_matrix': mtx, 'dist_coeff': dist, 'rms': rms}
         return rms, mtx, dist, rvecs, tvecs, objpoints, imgpoints

    def findMarkers(self, gray, objpoints, imgpoints):
            # objp = np.zeros((self.marker_size[0]*self.marker_size[1],3), np.float32)
            # objp[:,:2] = np.mgrid[0:self.marker_size[0],0:self.marker_size[1]].T.reshape(-1,2)
            objp = np.zeros((np.prod(self.marker_size), 3), np.float32)
            objp[:, :2] = np.indices(self.marker_size).T.reshape(-1, 2)  # make a grid of points

            # Find the chess board corners or circle centers
            if self.marker_type is Markers.checkerboard:
                flags = 0
                ret, corners = cv2.findChessboardCorners(gray, self.marker_size, flags=flags)
            elif self.marker_type is Markers.acircle:
                flags=cv2.CALIB_CB_ASYMMETRIC_GRID
                ret, corners = cv2.findCirclesGrid(gray, self.marker_size, flags=flags)
            else:
                raise Exception("invalid marker type: {}".format(self.marker_type))

            if ret:
                # rt = cv2.cornerSubPix(gray_l, corners_l, (11, 11),(-1, -1), self.criteria)
                imgpoints.append(corners.reshape(-1, 2))
                objpoints.append(objp)
            else:
                corners = [] # didn't find any

            return ret, objpoints, imgpoints, corners


class StereoCalibration(object):
    def __init__(self):
        self.camera_model = None

    def save(self, filename, handler=pickle):
        if self.camera_model is None:
            print("no camera model to save")
            return
        with open(filename, 'wb') as f:
            handler.dump(self.camera_model, f)

    def stereo_calibrate(self, imgs_l, imgs_r, marker_type, marker_size):
        cc = CameraCalibration()
        rms1, M1, d1, r1, t1, objpoints, imgpoints_l = cc.calibrate(imgs_l, marker_type, marker_size)
        rms2, M2, d2, r2, t2, objpoints, imgpoints_r = cc.calibrate(imgs_r, marker_type, marker_size)

        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)

        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            objpoints,
            imgpoints_l,
            imgpoints_r,
            M1, d1,
            M2, d2,
            imgs_l[0].shape[:2],
            criteria=stereocalib_criteria,
            flags=flags)

        print('Intrinsic_mtx_1', M1)
        print('dist_1', d1)
        print('Intrinsic_mtx_2', M2)
        print('dist_2', d2)
        print('R', R)
        print('T', T)
        print('E', E)
        print('F', F)

        # for i in range(len(r1)):
        #     print("--- pose[", i+1, "] ---")
        #     ext1, _ = cv2.Rodrigues(r1[i])
        #     ext2, _ = cv2.Rodrigues(r2[i])
        #     print('Ext1', ext1)
        #     print('Ext2', ext2)

        # print('')
        self.camera_model = {
            'date': time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()),
            'markerType': marker_type,
            'markerSize': marker_size,
            'imageSize': imgs_l[0].shape,
            'cameraMatrix1': M1,
            'cameraMatrix2': M2,
            'distCoeffs1': d1,
            'distCoeffs2': d2,
            # 'rvecs1': r1,
            # 'rvecs2': r2,
            'R': R,
            'T': T,
            'E': E,
            'F': F
        }

        return ret

if __name__ == '__main__':

    if False:
        path = 'checkerboard-imgs/*.png'
        marker = Markers.checkerboard
        dims = (7,10)
    else:
        path = 'acircle-imgs/*.png'
        marker = Markers.acircle
        dims = (4,11)

    scam = SCamera()
    imgs_l, imgs_r = scam.get_images(path)
    # scam.imshow(cc.save_cal_imgs, msec=1500)

    sc = StereoCalibration()
    ok = sc.stereo_calibrate(imgs_l, imgs_r, marker, dims)
    if ok:
        sc.save("camera_model.pickle")

        rec = Rectify("camera_model.pickle", alpha=1)
        cv2.imshow('image', rec.undistortLeft(imgs_l[3]))
        cv2.waitKey()
    else:
        print("Crap ... failure")
