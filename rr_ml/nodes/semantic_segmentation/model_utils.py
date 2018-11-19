import keras
import cv2
import numpy as np


class UNetModelUtils(object):
    def __init__(self, input_shape, n_classes):
        self.in_height = input_shape[0]
        self.in_width = input_shape[1]
        self.n_channels = input_shape[2]
        self.n_classes = n_classes

        assert self.in_height % 4 == 0
        assert self.in_width % 4 == 0

    # significant credit to https://github.com/divamgupta/image-segmentation-keras
    def make_unet_model(self):
        inputs = keras.layers.Input((self.in_height, self.in_width, self.n_channels))

        conv1 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(inputs)
        conv1 = keras.layers.Dropout(0.2)(conv1)
        conv1 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(conv1)

        pool1 = keras.layers.MaxPooling2D(pool_size=2)(conv1)

        conv2 = keras.layers.Conv2D(24, 3, activation='relu', padding='same')(pool1)
        conv2 = keras.layers.Dropout(0.2)(conv2)
        conv2 = keras.layers.Conv2D(24, 3, activation='relu', padding='same')(conv2)

        pool2 = keras.layers.MaxPooling2D(pool_size=2)(conv2)

        conv3 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(pool2)
        conv3 = keras.layers.Dropout(0.2)(conv3)
        conv3 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(conv3)

        up1 = keras.layers.UpSampling2D(size=2)(conv3)
        up1 = keras.layers.Concatenate(axis=3)([up1, conv2])

        conv4 = keras.layers.Conv2D(24, 3, activation='relu', padding='same')(up1)
        conv4 = keras.layers.Dropout(0.2)(conv4)
        conv4 = keras.layers.Conv2D(24, 3, activation='relu', padding='same')(conv4)

        up2 = keras.layers.UpSampling2D(size=2)(conv4)
        up2 = keras.layers.Concatenate(axis=3)([up2, conv1])

        conv5 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(up2)
        conv5 = keras.layers.Dropout(0.2)(conv5)
        conv5 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(conv5)

        conv6 = keras.layers.Conv2D(self.n_classes, 1, padding='same')(conv5)
        conv7 = keras.layers.Activation('softmax')(conv6)

        model = keras.Model(inputs=inputs, outputs=conv7)
        return model

    def load_input(self, fname):
        img = cv2.imread(fname)
        img = img.astype(np.float32) / 255.0
        return cv2.resize(img, (self.in_width, self.in_height))

    def load_label(self, fname):
        img = np.load(fname)
        label_img = np.zeros(img.shape[:2] + (self.n_classes,), dtype=np.float32)
        for r in range(img.shape[0]):
            for c in range(img.shape[1]):
                chan = img[r, c]
                label_img[r, c, chan] = 1.0
        return cv2.resize(label_img, (self.in_width, self.in_height))

    def prediction_to_image(self, y):
        colors = [(0, 0, 0), (255, 0, 0), (0, 255, 0)]
        my = np.argmax(y, axis=-1)
        img = np.ndarray((self.in_height, self.in_width, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            mask = (my == i).reshape(img.shape[:2])
            img[mask] = color
        return img


def IOU_score(y1, y2):
    argmax_img1 = np.argmax(y1, axis=-1)
    argmax_img2 = np.argmax(y2, axis=-1)
    mask = np.logical_or((argmax_img1 != 0), (argmax_img2 != 0))

    total = np.sum(mask)
    right = np.sum(np.equal(argmax_img1, argmax_img2)[mask])
    return float(right) / total
