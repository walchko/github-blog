#!/usr/bin/env python3

import cv2
import time

# 0: black image
# color image: 1 left (as you look at it)
# usb3: (720, 2560, 3) 0
camera = cv2.VideoCapture(0)

cnt = 0

while True:
    ok, frame = camera.read()
    if ok:
        print(frame.shape)
        h, w, d = frame.shape
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.resize(frame, (w//2,h//2))
        print(frame.shape)
        # frame = cv2.equalizeHist(frame)
        cv2.imshow("img", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('image-{}.png'.format(cnt), frame)
            cnt += 1
        time.sleep(1/10)
    else:
        print(">> nothing: {}".format(ok))
        time.sleep(1)
camera.release()
