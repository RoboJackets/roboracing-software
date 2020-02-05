import rospy
from . import util


def main():
    rospy.init_node("image_labeler_single")

    img_path = rospy.get_param("~image_path")
    label_path = rospy.get_param("~label_path")

    cfg_file = rospy.get_param("~config_file")

    util.label_image(img_path, label_path, cfg_file)
