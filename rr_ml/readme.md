# RR_ML
RoboRacing's machine learning package

## External Dependencies
Installed via pip:
- keras
- tensorflow-gpu (non-gpu version works but is much slower)
- numpy
- labelme
- pyyaml

`sudo python -m pip install keras tensorflow-gpu tensorboard numpy labelme pyyaml` \
OR \
`python -m pip install --user keras tensorflow-gpu tensorboard numpy labelme pyyaml`


## Image Labeling Pipeline

1. Extract image data from bag file: 
`roslaunch rr_ml extract_images_iarrc_lines.launch bag_file:=/path/to/bag image_topic:=/topic num_images:=30`
1. Label all unlabeled images from iarrc_lines dataset: `roslaunch label_folder_iarrc_lines.launch`
    1. Ctrl+S to save, next image will open automatically
    1. Close the window and/or Ctrl-C in the terminal to cancel labeling (will resume from same point next time)

## Training script

- Train segmentation network: `roslaunch rr_ml train_seg_iarrc_lines.launch epochs:=1000`
- In separate terminal: `tensorboard --logdir rr_ml/tensorboard_logs/ --host localhost --port 6006`
- Training script will stop after 'epochs' epochs or when the loss on the validation set stops going down
