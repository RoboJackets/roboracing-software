import random
import os
import sys

import cv2
import numpy as np


if "-n" in sys.argv:
    i = sys.argv.index("-n")
    n_imgs = int(sys.argv[i + 1])
else:
    n_imgs = 100

for n in range(n_imgs):
    img = np.zeros((180, 320, 3), dtype=np.uint8)
    rows, cols, channels = img.shape
    n_categories = 3

    colors = [(200, 100, 50), (50, 100, 200), (60, 90, 210)]

    img[:,:,:] = colors[0]
    cv2.circle(img, (random.randint(50, 270), random.randint(50, 130)), random.randint(20, 50), colors[1],
               thickness=-1)
    cv2.circle(img, (random.randint(50, 270), random.randint(50, 130)), random.randint(20, 50), colors[2],
               thickness=-1)

    def get_mask_for(color):
        mask = (img == color)  # mask 3D
        mask = mask.all(axis=2)  # mask 2D
        return mask

    label = np.zeros((rows, cols), dtype=np.uint8)
    label[get_mask_for(colors[1])] = 1
    label[get_mask_for(colors[2])] = 2

    img = (img + np.random.randint(-50, 50, img.shape)).astype(np.uint8)

    if not os.path.exists("data"):
        os.makedirs("data")

    cv2.imwrite("data/test_img_%d.png" % n, img)
    cv2.imwrite("data/test_label_%d.png" % n, label)

    # cv2.imshow("data", img)
    # cv2.imshow("label", label * 50)
    # cv2.waitKey(10)
