
import numpy as np
import cv2
from glob import glob


class Rectify(object):
    """
    Loads camera calibration info, either mono or stereo, and uses the parameters
    to rectify images.
    """
    def __init__(self, filename, alpha=0.5):
        self.info = self.__read(filename)
        self.alpha = alpha # 0=full crop, 1=no crop
        self.maps_read = False

#         # use stereoRectify to calculate what we need to rectify stereo images
#         M1 = self.info["cameraMatrix1"]
#         d1 = self.info["distCoeffs1"]
#         M2 = self.info["cameraMatrix2"]
#         d2 = self.info["distCoeffs2"]
#         size = self.info['size']
#         R = self.info['R']
#         T = self.info['T']
#         R1, R2, self.P1, self.P2, self.Q, roi1, roi2 = cv2.stereoRectify(M1, d1, M2, d2, size, R, T, alpha=alpha)

#         # these return undistortion and rectification maps which are both stored in maps_x for
#         # camera 1 and 2
#         self.maps_1 = cv2.initUndistortRectifyMap(M1, d1, R1, P1, size, cv2.CV_16SC2)  # CV_32F?
#         self.maps_2 = cv2.initUndistortRectifyMap(M2, d2, R2, P2, size, cv2.CV_16SC2)

    def __read(self, filename, handler=pickle):
        with open(filename, 'rb') as f:
            data = handler.load(f)
        # print(data)
        return data

    # def __fix2(self, image, maps, inter=cv2.INTER_LANCZOS4):
    #     return cv2.remap(image, maps[0], maps[1], inter)

    # def __fix(self, image, m, d):
    #     """
    #     image: an image
    #     alpha = 0: returns undistored image with minimum unwanted pixels (image
    #                 pixels at corners/edges could be missing)
    #     alpha = 1: retains all image pixels but there will be black to make up
    #                 for warped image correction
    #     """
    #     h,w = image.shape[:2]
    #     # Adjust the calibrations matrix
    #     # alpha=0: returns undistored image with minimum unwanted pixels
    #     #           (image pixels at corners/edges could be missing)
    #     # alpha=1: retains all image pixels but there will be black to make
    #     #           up for warped image correction
    #     # returns new cal matrix and an ROI to crop out the black edges
    #     newcameramtx, _ = cv2.getOptimalNewCameraMatrix(m, d, (w, h), self.alpha)
    #     # undistort
    #     ret = cv2.undistort(image, m, d, None, newcameramtx)
    #     return ret

    def __mono(self, image):
        """
        image: an image
        alpha = 0: returns undistored image with minimum unwanted pixels (image
                    pixels at corners/edges could be missing)
        alpha = 1: retains all image pixels but there will be black to make up
                    for warped image correction
        """
        mtx = self.info['cameraMatrix']
        dist = self.info['distCoeffs']
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


    def undistortStereo(self, left, right=None):
        if right is None:
            return self.__mono(left)
        return self.__stereo(left, right)

    def __stereo(self, left, right):
        # return self.undistortLeft(left), self.undistortRight(right)
        if not self.maps_read:
            # clear init flag
            self.maps_read = True
            # use stereoRectify to calculate what we need to rectify stereo images
            M1 = self.info["cameraMatrix1"]
            d1 = self.info["distCoeffs1"]
            M2 = self.info["cameraMatrix2"]
            d2 = self.info["distCoeffs2"]
            size = self.info['imageSize']
            R = self.info['R']
            T = self.info['T']
            h, w = size[:2]
            R1, R2, self.P1, self.P2, self.Q, roi1, roi2 = cv2.stereoRectify(M1, d1, M2, d2, (w,h), R, T, alpha=self.alpha)

            # these return undistortion and rectification maps which are both
            # stored in maps_x for camera 1 and 2
            self.maps_1 = cv2.initUndistortRectifyMap(M1, d1, R1, self.P1, (w,h), cv2.CV_16SC2)  # CV_32F?
            self.maps_2 = cv2.initUndistortRectifyMap(M2, d2, R2, self.P2, (w,h), cv2.CV_16SC2)

        # return self.__fix2(left, self.maps_1), self.__fix2(right, self.maps_2)
        inter=cv2.INTER_LANCZOS4
        return cv2.remap(left, self.maps_1[0], self.maps_1[1], inter),
            cv2.remap(right, self.maps_2[0], self.maps_2[1], inter)

    def printParams(self):
        if self.info is None:
            print("No camera calibration info loaded")
            return

        if "cameraMatrix2" in self.info:
            print("stereo info")
        else:
            print("mono info")

    def project3d(self, disparity):
        if not self.maps_read:
            print('*** WARNING: You need to call undistortStereo() first so a Q matrix is calculated ***')
            return None
        return cv2.reprojectImageTo3D(disparity, self.Q)


class MonoCamera(object):
    def imshow(self, imgs, scale=3, msec=500):
        for i, img in enumerate(imgs):
            h,w = img.shape[:2]
            cv2.imshow('image-{}'.format(i), cv2.resize(img, (w//scale,h//scale)))
            cv2.waitKey(msec)

    def get_images(self, path, gray=False):
        """
        Given a path, it reads all images. This uses glob to grab file names
        and excepts wild cards *
        Ex. getImages('./images/*.jpg')
        """
        imgs = []
        files = glob(path)

        print("Found {} images at {}".format(len(tuple(files)), path))
        # print('-'*40)

        for i, f in enumerate(files):
            img = cv2.imread(f)
            if img is None:
                print('>> Could not read: {}'.format(f))
            else:
                if gray and len(img.shape) > 2:
                   img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                print("[{}]:{} ({}, {})".format(i, f, *img.shape))
                h, w = img.shape[:2]
                imgs.append(img)
        # print('-'*40)
        return imgs


class StereoCamera(object):
    """
    rename StereoCamera or EX8029

    This is for the eYs3D Stereo Camera - EX8029 which can be purchased from
    https://www.sparkfun.com/products/14726
    """
    def imshow(self, imgs, scale=3, msec=500):
        for i, img in enumerate(imgs):
            h,w = img.shape[:2]
            cv2.imshow('image-{}'.format(i), cv2.resize(img, (w//scale,h//scale)))
            cv2.waitKey(msec)

    def get_images(self, path, gray=False):
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
            img = cv2.imread(f)
            if img is None:
                print('>> Could not read: {}'.format(f))
            else:
                h, w = img.shape[:2]

                if gray:
                    if len(img.shape) > 2:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    l = img[:, :w//2]
                    r = img[:, w//2:]
                else:
                    l = img[:, :w//2, :]
                    r = img[:, w//2:, :]

                imgs_l.append(l)
                imgs_r.append(r)
        # print('-'*40)
        return imgs_l, imgs_r
