import subprocess
import json
import tempfile

import labelme
import cv2
import numpy as np
import yaml


acceptable_image_formats = (".jpg", ".jpeg", ".bmp", ".png")


# given a string in yaml dictionary format, return a dict with the background entry added
def get_label_name_to_value(config_file):
    with open(config_file) as f:
        cfg = yaml.load(f)

    label_name_to_value = dict()
    label_name_to_value["_background"] = 0

    for i, label in enumerate(cfg['labels']):
        label_name_to_value[label] = i + 1

    return label_name_to_value


def label_image(img_path, label_path, cfg_file):
    name_to_value = get_label_name_to_value(cfg_file)

    with tempfile.NamedTemporaryFile(suffix=".json") as json_file:
        subprocess.call(["labelme", img_path, "--config", cfg_file, "--output", json_file.name])

        with open(json_file.name) as f:
            json_data = json.load(f)

        img = cv2.imread(img_path)

        label = labelme.utils.shapes_to_label(img.shape, json_data['shapes'], name_to_value)
        label = label.astype(np.uint8)
        np.save(label_path, label)
