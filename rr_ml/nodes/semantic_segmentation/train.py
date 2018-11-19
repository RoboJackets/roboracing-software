import os
import glob
import random
import time

import cv2
import keras
import numpy as np
import rospy

from . import model_utils

osp = os.path

batch_size = 1


def data_gen(helper, input_files, label_files):
    b = 0
    example_input = helper.load_input(input_files[0])
    example_label = helper.load_label(label_files[0])

    while b + batch_size <= len(input_files):
        inputs = np.zeros((batch_size,) + example_input.shape, dtype=np.float32)
        labels = np.zeros((batch_size,) + example_label.shape, dtype=np.float32)

        for n in range(batch_size):
            inputs[n] = helper.load_input(input_files[b + n])
            labels[n] = helper.load_label(label_files[b + n])

        b += batch_size
        yield inputs, labels


def data_gen_wrapper(helper, input_files, label_files, epochs):
    for epoch in range(epochs):
        for x in data_gen(helper, input_files, label_files):
            yield x


def main(acceptable_image_formats):
    rospy.init_node("train_segmentation")

    data_dir = rospy.get_param("~data_dir")
    model_path = rospy.get_param("~model_path")

    n_epochs = rospy.get_param("~epochs")
    validate_only = rospy.get_param("~validate_only")
    visualize_validation = rospy.get_param("~visualize_validation")

    input_files = []
    for fmt in acceptable_image_formats:
        input_files += glob.glob(osp.join(data_dir, "*" + fmt))
    input_files.sort()

    label_files = sorted(glob.glob(osp.join(data_dir, "*.npy")))

    files = list(zip(input_files, label_files))
    random.shuffle(files)
    split = int(len(input_files) * 0.75)
    training_files = files[:split]
    validate_files = files[split:]

    training_input_files, training_label_files = zip(*training_files)
    validate_input_files, validate_label_files = zip(*validate_files)

    unet_helper = model_utils.UNetModelUtils(input_shape=(216, 384, 3), n_classes=3)

    try:
        model = keras.models.load_model(model_path)
        print "loaded model from disk"
    except IOError:
        model = unet_helper.make_unet_model()
        model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
        print "no model found on disk, created a new one"
        time.sleep(1.0)

    model.summary()

    if not validate_only:
        model.fit_generator(data_gen_wrapper(unet_helper, training_input_files, training_label_files, n_epochs),
                            steps_per_epoch=len(training_files) // batch_size, epochs=n_epochs, verbose=1,
                            max_queue_size=5)

        if not osp.exists(osp.dirname(model_path)):
            os.makedirs(osp.dirname(model_path))
        keras.models.save_model(model, model_path)
        print "saved to disk"

        loss, accuracy = model.evaluate_generator(data_gen(unet_helper, validate_input_files, validate_label_files),
                                                  steps=len(validate_files) // batch_size, verbose=1)
        print "validation loss", loss, "accuracy", accuracy

    scores = []
    pred_times = []
    for fx, fy in zip(validate_input_files, validate_label_files):
        x = unet_helper.load_input(fx)
        y_ = unet_helper.load_label(fy)

        t_pred_0 = time.time()
        y = model.predict(x[None])[0]
        t_pred = time.time() - t_pred_0
        pred_times.append(t_pred)

        iou_score = model_utils.IOU_score(y, y_)
        scores.append(iou_score)

        if visualize_validation:
            cv2.imshow("input", (x * 255).astype(np.uint8))
            cv2.imshow("ground truth", unet_helper.prediction_to_image(y_))
            cv2.imshow("output", unet_helper.prediction_to_image(y))

            while True:
                k = cv2.waitKey(1)
                if k == ord(' '):
                    break
                if k == ord('q'):
                    visualize_validation = False
                    break

    print "avg IOU =", sum(scores) / len(scores)
    print "avg prediction time", sum(pred_times) / len(pred_times)
