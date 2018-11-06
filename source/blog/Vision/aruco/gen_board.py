#!/usr/bin/env python3
import cv2
from cv2 import aruco

dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)


x = 8  # horizontal
y = 11  # vertical
sqr = 0.1  # solid black squares
mrk = 0.05 # markers, must be smaller than squares
board = aruco.CharucoBoard_create(x,y,sqr,mrk,dictionary)

sz = (1024, 1024)
brdbits = 1
img = board.draw(sz, None, 0, brdbits)

cv2.imshow('marker', img)
key = cv2.waitKey()

if key == ord('s'):
    cv2.imwrite('board.png', img)
