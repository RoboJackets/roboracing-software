"""
gen_fake_data.py

Generate random images for testing a segmentation network. This script
will place 2 circles of slightly different colors on a background in each
image.
"""

import random
import os
import sys

import cv2
import numpy as np


assert "-n" in sys.argv
i = sys.argv.index("-n")
n_imgs = int(sys.argv[i + 1])


DIR = os.path.dirname(__file__)
data_dir = os.path.join(DIR, "../../data/segmentation/test_circles")


colors = [(100, 100, 100), (90, 100, 110), (100, 110, 90)]
n_categories = len(colors)
circle_size_range = (20, 50)

local_noise_range = (-20, 20)
global_noise_range = (-60, 60)


def random_feature_circle(img, color):
    max_r = circle_size_range[1]
    rows, cols = img.shape[:2]
    img = cv2.circle(img,
               (random.randint(max_r, cols - max_r), random.randint(max_r, rows - max_r)),
               random.randint(*circle_size_range),
               color,
               thickness=-1)
    return img


for n in range(n_imgs):
    img = np.zeros((180, 320, 3), dtype=np.uint8)
    rows, cols, channels = img.shape

    img[:,:,:] = colors[0]

    img = random_feature_circle(img, colors[1])
    img = random_feature_circle(img, colors[2])

    def get_mask_for(color):
        mask = (img == color)  # mask 3D
        mask = mask.all(axis=2)  # mask 2D
        return mask

    label = np.zeros((rows, cols), dtype=np.uint8)
    label[get_mask_for(colors[1])] = 1
    label[get_mask_for(colors[2])] = 2

    local_noise_mat = np.random.randint(local_noise_range[0], local_noise_range[1], img.shape)
    global_noise_mat = np.ones(img.shape, dtype=np.int16) * np.random.randint(*global_noise_range)
    img = (img + local_noise_mat + global_noise_mat).astype(np.uint8)

    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    cv2.imwrite(os.path.join(data_dir, "input_%d.png" % n), img)
    np.save(os.path.join(data_dir, "label_%d.npy" % n), label)
