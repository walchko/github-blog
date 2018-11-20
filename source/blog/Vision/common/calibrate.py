#!/usr/bin/env python3
import numpy as np
import cv2
from glob import glob
# import argparse
from enum import Enum
import time
import pickle
# import cv2.aruco as aruco

# change these? circles asym_circles ???
Markers = Enum('Markers', 'checkerboard circle acircle aruco charuco')

# class PLY(object):
#     """
#     This is a modified version of the code in an opencv tutorial
#     """
#     ply_header = """ply
# format ascii 1.0
# element vertex {}
# property float x
# property float y
# property float z
# property uchar red
# property uchar green
# property uchar blue
# end_header
# """
#
#     def write(self, fname, verts, colors):
#         verts = verts.reshape(-1, 3)
#         colors = colors.reshape(-1, 3)
#         verts = np.hstack([verts, colors])
#         with open(fname, 'w') as f:
#             f.write(self.ply_header.format(len(verts)))
#             np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')




class CameraCalibration(object):
    def __init__(self):
        # self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        # x = 5  # horizontal
        # y = 7  # vertical
        # sqr = 0.254  # solid black squares
        # mrk = 0.23 # markers, must be smaller than squares
        # self.board = aruco.CharucoBoard_create(x,y,sqr,mrk,self.dictionary)

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    def save(self, filename, handler=pickle):
        with open(filename, 'wb') as f:
            handler.dump(self.info, f)

    def drawAxes(img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img

    def calibrate(self, images, marker_type, marker_size, marker_scale=1):
         """
         images: an array of grayscale images, all assumed to be the same size
         marker_scale: how big are your markers in the real world, example:
            checkerboard with sides 2 cm, set marker_scale=0.02 so your T matrix
            comes out in meters
         """
         self.marker_type = marker_type
         self.marker_size = marker_size
         self.save_cal_imgs = []
         self.marker_scale = marker_scale

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

                 # draw the axes
                 # self.drawAxes()
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
             'imageSize': images[0].shape,
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
            objp[:, :2] = np.indices(self.marker_size).T.reshape(-1, 2)*self.marker_scale  # make a grid of points

            # Find the chess board corners or circle centers
            if self.marker_type is Markers.checkerboard:
                flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
                ret, corners = cv2.findChessboardCorners(gray, self.marker_size, flags=flags)
            elif self.marker_type is Markers.circle:
                flags=0
                ret, corners = cv2.findCirclesGrid(gray, self.marker_size, flags=flags)
            elif self.marker_type is Markers.acircle:
                flags=cv2.CALIB_CB_ASYMMETRIC_GRID
                ret, corners = cv2.findCirclesGrid(gray, self.marker_size, flags=flags)
            # elif self.marker_type is Markers.charuco:
            #     corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dictionary)
            #     # ret = True if len(ids) > 0 else False
            #     if len(ids) > 0:
            #         ret, corners, ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
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
        self.save_cal_imgs = None

    def save(self, filename, handler=pickle):
        if self.camera_model is None:
            print("no camera model to save")
            return
        with open(filename, 'wb') as f:
            handler.dump(self.camera_model, f)

    def stereo_calibrate(self, imgs_l, imgs_r, marker_type, marker_size, marker_scale=1):
        """
        This will save the found markers for camera_2 (right) only in
        self.save_cal_imgs array
        """
        cc = CameraCalibration()
        rms1, M1, d1, r1, t1, objpoints, imgpoints_l = cc.calibrate(
            imgs_l, marker_type, marker_size, marker_scale)
        rms2, M2, d2, r2, t2, objpoints, imgpoints_r = cc.calibrate(
            imgs_r, marker_type, marker_size, marker_scale)
        self.save_cal_imgs = cc.save_cal_imgs

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

        print('-'*50)
        print('Image: {}x{}'.format(*imgs_l[0].shape[:2]))
        print('{}: {}'.format(marker_type, marker_size))
        print('Intrinsic Camera Parameters')
        print('-'*50)
        print(' [Camera 1]')
        # print('  cameraMatrix_1', M1)
        print('  f(x,y): {:.1f} {:.1f} px'.format(M1[0,0], M1[1,1]))
        print('  principlePoint(x,y): {:.1f} {:.1f} px'.format(M1[0,2], M1[1,2]))
        print('  distCoeffs', d1[0])
        print(' [Camera 2]')
        # print('  cameraMatrix_2', M2)
        print('  f(x,y): {:.1f} {:.1f} px'.format(M2[0,0], M2[1,1]))
        print('  principlePoint(x,y): {:.1f} {:.1f} px'.format(M2[0,2], M2[1,2]))
        print('  distCoeffs', d2[0])
        print('-'*50)
        print('Extrinsic Camera Parameters')
        print('-'*50)
        print('  R', R)
        print('  T[meter]', T)
        print('  E', E)
        print('  F', F)

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
            'imageSize': imgs_l[0].shape[:2],
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
#
# if __name__ == '__main__':
#
#     if False:
#         path = 'checkerboard-imgs/*.png'
#         marker = Markers.checkerboard
#         dims = (7,10)
#         scale = 0.02  # 2 cm on each side
#         fname = 'cb_camera_model.pickle'
#     elif True:
#         path = 'aruco-imgs/*.png'
#         marker = Markers.charuco
#         dims = (5,7)
#         scale = 0.254  # 1 in or 2.54 cm on each side
#         fname = 'charuo_camera_model.pickle'
#     else:
#         path = 'acircle-imgs/*.png'
#         marker = Markers.acircle
#         dims = (4,11)
#         scale = 0.01  # 1 cm diameter
#         fname = 'ac_camera_model.pickle'
#
#     scam = SCamera()
#     imgs_l, imgs_r = scam.get_images(path, gray=True)
#
#     sc = StereoCalibration()
#     ok = sc.stereo_calibrate(imgs_l, imgs_r, marker, dims, marker_scale=scale)
#     if ok:
#         sc.save(fname)
#
#         # scam.imshow(sc.save_cal_imgs, msec=1500)
#
#         rec = Rectify(fname, alpha=1)
#         l, r = rec.undistortStereo(imgs_l[0],imgs_r[0])
#
#         # for l,r in zip(imgs_l, imgs_r):
#         #     l,r = rec.undistortStereo(l,r)
#         #     h = np.hstack((l,r))
#         #     cv2.imshow('image', h)
#         #     cv2.waitKey(1000)
#     else:
#         print("Crap ... failure")
