import os
import glob
import random
import time

import cv2
import keras
import numpy as np
import rospy
import tensorflow as tf

from . import model_utils

osp = os.path
K = keras.backend

batch_size = 8

load_cache = dict()
max_load_cache_entries = 5000


def data_gen(helper, input_files, label_files, do_augmentation):
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
            f_in = input_files[b + n]
            if f_in in load_cache:
                ipt = load_cache[f_in]
            else:
                ipt = helper.load_input(f_in)
                if len(load_cache) < max_load_cache_entries:
                    load_cache[f_in] = ipt

            f_lbl = label_files[b + n]
            if f_lbl in load_cache:
                lbl = load_cache[f_lbl]
            else:
                lbl = helper.load_label(f_lbl)
                if len(load_cache) < max_load_cache_entries:
                    load_cache[f_lbl] = lbl

            if do_augmentation:

                # left/right flips
                if random.random() < 0.5:
                    ipt = np.fliplr(ipt)
                    lbl = np.fliplr(lbl)

                # lighting fluctuations
                ipt[:, :, 0] += random.normalvariate(0, 0.08)  # Hue
                ipt[:, :, 1] += random.normalvariate(0, 0.15)  # Lightness
                ipt[:, :, 2] += random.normalvariate(0, 0.15)  # Saturation

                # crops
                if random.random() < 0.5:
                    prev_shape = ipt.shape
                    direction = random.randint(0, 3)
                    amount = random.randint(1, 100)
                    if direction == 0:
                        ipt = ipt[amount:]
                        lbl = lbl[amount:]
                    elif direction == 1:
                        ipt = ipt[:-amount]
                        lbl = lbl[:-amount]
                    elif direction == 2:
                        ipt = ipt[:, amount:]
                        lbl = lbl[:, amount:]
                    else:
                        ipt = ipt[:, :-amount]
                        lbl = lbl[:, :-amount]

                    ipt = cv2.resize(ipt, (prev_shape[1], prev_shape[0]))
                    lbl = cv2.resize(lbl, (prev_shape[1], prev_shape[0]))

            inputs[n] = ipt
            labels[n] = lbl

        b += batch_size
        yield inputs, labels


def data_gen_wrapper(helper, input_files, label_files, epochs, do_augmentation):
    for epoch in range(epochs):
        for x in data_gen(helper, input_files, label_files, do_augmentation):
            yield x


def focal_loss(gamma):
    def loss(target, output):
        output /= K.sum(output, axis=-1, keepdims=True)
        eps = K.epsilon()
        output = K.clip(output, eps, 1. - eps)

        focal_loss_pos = -target * K.log(output) * K.pow(1. - output, gamma)
        # focal_loss_neg = (target - 1.) * K.log(1. - output) * K.pow(output, gamma)
        focal_loss_neg = 0
        return K.sum(focal_loss_pos + focal_loss_neg, axis=-1)
    return loss


def single_iou_score(y1, y2):
    argmax_img1 = np.argmax(y1, axis=-1)
    argmax_img2 = np.argmax(y2, axis=-1)
    mask = ((argmax_img1 + argmax_img2) > 0)

    total = np.sum(mask)
    right = np.sum(np.equal(argmax_img1, argmax_img2)[mask])
    return (float(right) / total) if total > 0 else 0


def iou(y_true, y_pred):
    def f(y1, y2):
        y1 = y1.numpy()
        y2 = y2.numpy()
        scores = np.zeros(y1.shape[0])
        for i in range(y1.shape[0]):
            scores[i] = single_iou_score(y1[i], y2[i])
        return scores

    return tf.py_function(f, [y_true, y_pred], tf.double)


def get_save_function(model, model_path):
    def save():
        model.save(model_path, include_optimizer=False)
        print "saved to disk"
    return save


def main(acceptable_image_formats):
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.7
    keras.backend.set_session(tf.Session(config=config))

    rospy.init_node("train_segmentation")

    data_dir = rospy.get_param("~data_dir")
    model_path = rospy.get_param("~model_path")
    n_epochs = rospy.get_param("~epochs")

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

    unet_helper = model_utils.UNetModelUtils()

    try:
        model = keras.models.load_model(model_path)
        print "loaded model from disk"
    except IOError:
        model = unet_helper.make_unet_model()
        print "no model found on disk, created a new one"

    model.compile(loss=focal_loss(2),
                  optimizer="adam",
                  metrics=[iou])
    time.sleep(2.0)
    model.summary()

    save = get_save_function(model, model_path)

    data = data_gen_wrapper(unet_helper, training_input_files, training_label_files, n_epochs, True)
    data_validate = data_gen_wrapper(unet_helper, validate_input_files, validate_label_files, n_epochs, False)

    steps_per_epoch = len(training_files) // batch_size
    validation_steps = len(validate_files) // batch_size

    def on_epoch_end(epoch, log):
        if epoch != 0 and epoch % 20 == 0:
            save()

    tensorboard_log_dir = osp.join(osp.dirname(__file__), '../../tensorboard_logs')
    for fname in os.listdir(tensorboard_log_dir):
        fname = osp.join(tensorboard_log_dir, fname)
        while osp.exists(fname):
            os.remove(fname)

    callbacks = [
        keras.callbacks.EarlyStopping(monitor='val_loss', min_delta=0, patience=200, verbose=1, mode='min'),
        keras.callbacks.TensorBoard(log_dir=tensorboard_log_dir,
                                    write_graph=True, write_grads=True, write_images=False, update_freq='epoch'),
        keras.callbacks.LambdaCallback(on_epoch_end=on_epoch_end)
    ]

    if not osp.exists(osp.dirname(model_path)):
        os.makedirs(osp.dirname(model_path))

    model.fit_generator(data, steps_per_epoch=steps_per_epoch, epochs=n_epochs, verbose=1,
                        validation_data=data_validate, validation_steps=validation_steps, callbacks=callbacks)

    save()
