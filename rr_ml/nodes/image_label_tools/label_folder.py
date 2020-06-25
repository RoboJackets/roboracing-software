import os

import rospy
from . import util

osp = os.path


def main():
    rospy.init_node("image_labeler")

    image_dir = rospy.get_param("~image_dir")
    label_dir = rospy.get_param("~label_dir")

    cfg_file = rospy.get_param("~config_file")

    print "using image dir:", image_dir
    print "using label dir:", label_dir

    img_names = [f for f in os.listdir(image_dir) if osp.splitext(f)[1] in util.acceptable_image_formats]
    img_names.sort()

    for img_name in img_names:
        img_path = osp.join(image_dir, img_name)

        f = osp.splitext(img_name)[0]
        label_path = osp.join(label_dir, f + ".npy")

        if not os.path.exists(label_path):
            util.label_image(img_path, label_path, cfg_file)
