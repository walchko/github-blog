#!/usr/bin/env python3
import imageio
from glob import glob


def save(imgFiles, fname, fps=30):
    writer = imageio.get_writer(fname, fps=fps)
    sz = len(imgFiles)
    for i, imgf in enumerate(imgFiles):
        im = imageio.imread(imgf)
        # writer.append_data(im[:, :, 1])
        writer.append_data(im)
        if i%100 == 0:
            print(f">> Frame {i}/{sz}")
    writer.close()
    
mav_path = "/Users/kevin/tmp/mav/mav0/cam0/data/*.png"
imgfiles = sorted(glob(mav_path))
# print(imgfiles)
save(imgfiles, "test.mp4")