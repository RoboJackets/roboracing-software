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
K = keras.backend

batch_size = 8


def data_gen(helper, input_files, label_files):
    z = list(zip(input_files, label_files))
    random.shuffle(z)
    input_files, label_files = zip(*z)

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


def data_gen_wrapper(helper, input_files, label_files, epochs, cached=False):
    if cached:
        cache = list(data_gen(helper, input_files, label_files))
        print "cached training data"
        for epoch in range(epochs):
            random.shuffle(cache)
            for x in cache:
                yield x
    else:
        for epoch in range(epochs):
            for x in data_gen(helper, input_files, label_files):
                yield x


def weighted_focal_loss(gamma, class_imbalance):
    def loss(target, output):
        weights = [class_imbalance ** 0.5] * K.int_shape(output)[-1]
        weights[0] = 1
        weights = K.constant(weights)

        output /= K.sum(output, axis=-1, keepdims=True)
        eps = K.epsilon()
        output = K.clip(output, eps, 1. - eps)

        focal_loss_pos = -target * K.log(output) * K.pow(1. - output, gamma) * weights
        focal_loss_neg = (target - 1.) * K.log(1. - output) * K.pow(output, gamma) * (1. / weights)
        return K.sum(focal_loss_pos + focal_loss_neg, axis=-1)
    return loss


def max_output_non_background(y_true, y_pred):
    return K.mean(K.max(y_pred[:,:,:,1:], axis=[1,2,3]))


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
    # training_files = files[:]
    # validate_files = files[:]

    training_input_files, training_label_files = zip(*training_files)
    validate_input_files, validate_label_files = zip(*validate_files)

    unet_helper = model_utils.UNetModelUtils(input_shape=(144, 256, 3), n_classes=4)

    try:
        model = keras.models.load_model(model_path)
        print "loaded model from disk"
    except IOError:
        model = unet_helper.make_unet_model()
        print "no model found on disk, created a new one"

    model.compile(loss=weighted_focal_loss(3, 100), optimizer='adam', metrics=['accuracy', max_output_non_background])
    time.sleep(2.0)
    model.summary()

    if not validate_only:
        starting_weights = model.get_weights()
        print "warming up optimizer..."
        warmup_length = 20
        data = data_gen_wrapper(unet_helper, training_input_files, training_label_files, warmup_length, cached=True)
        model.fit_generator(data, steps_per_epoch=warmup_length, epochs=1, verbose=1)
        print "done warming up"
        model.set_weights(starting_weights)

        data = data_gen_wrapper(unet_helper, training_input_files, training_label_files, n_epochs, cached=True)
        model.fit_generator(data, steps_per_epoch=len(training_files) // batch_size, epochs=n_epochs, verbose=1)

        if not osp.exists(osp.dirname(model_path)):
            os.makedirs(osp.dirname(model_path))
        model.save(model_path, include_optimizer=False)
        print "saved to disk"

        metrics = model.evaluate_generator(data_gen(unet_helper, validate_input_files, validate_label_files),
                                           steps=len(validate_files) // batch_size, verbose=1)
        print "metrics", metrics

    scores = []
    pred_times = []
    for fx, fy in zip(validate_input_files, validate_label_files):
        x = unet_helper.load_input(fx)
        y_ = unet_helper.load_label(fy)

        t_pred_0 = time.time()
        y = model.predict(x[None])[0]
        t_pred = time.time() - t_pred_0
        pred_times.append(t_pred)

        maxes = [np.max(y[...,i]) for i in range(y.shape[-1])]
        print "max per category:", maxes

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
