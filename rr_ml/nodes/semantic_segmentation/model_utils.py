import keras
import cv2
import numpy as np


class UNetModelUtils(object):
    def __init__(self, input_shape):
        self.in_height = input_shape[0]
        self.in_width = input_shape[1]

        assert len(input_shape) == 2

        self.n_channels = 3
        self.n_classes = 2

        assert self.in_height % 4 == 0
        assert self.in_width % 4 == 0

    # significant credit to https://github.com/divamgupta/image-segmentation-keras
    def make_unet_model(self):
        L = keras.layers

        input_layer = L.Input((self.in_height, self.in_width, self.n_channels))

        n = 64
        conv1 = L.Conv2D(n, 3, activation='relu', padding='same')(input_layer)
        # conv1 = L.Dropout(0.05)(conv1)
        # conv1 = L.Conv2D(n, 3, activation='relu', padding='same')(conv1)

        pool1 = L.MaxPooling2D(pool_size=4)(conv1)

        n = 16
        conv2 = L.Conv2D(n, 5, activation='relu', padding='same')(pool1)
        # # conv2 = L.Dropout(0.05)(conv2)
        # # conv2 = L.Conv2D(n, 3, activation='relu', padding='same')(conv2)
        #
        # pool2 = L.MaxPooling2D(pool_size=2)(conv2)
        #
        # n = 8
        # conv3 = L.Conv2D(n, 3, activation='relu', padding='same')(pool2)
        # # conv3 = L.Dropout(0.05)(conv3)
        # # conv3 = L.Conv2D(n, 3, activation='relu', padding='same')(conv3)
        #
        up1 = L.UpSampling2D(size=4)(conv2)
        up1 = L.Concatenate(axis=3)([up1, conv1])
        #
        # n = 8
        # conv4 = L.Conv2D(n, 3, activation='relu', padding='same')(up1)
        # # conv4 = L.Dropout(0.05)(conv4)
        # # conv4 = L.Conv2D(n, 3, activation='relu', padding='same')(conv4)
        #
        # up2 = L.UpSampling2D(size=2)(conv4)
        # up2 = L.Concatenate(axis=3)([up2, conv1])
        #
        # n = 8
        # conv5 = L.Conv2D(n, 3, activation='relu', padding='same')(up2)
        # conv5 = L.Dropout(0.05)(conv5)
        # conv5 = L.Conv2D(n, 3, activation='relu', padding='same')(conv5)

        conv6 = L.Conv2D(self.n_classes, 1, padding='same')(up1)
        conv7 = L.Activation('softmax')(conv6)

        model = keras.Model(inputs=input_layer, outputs=conv7)
        return model

    def crop(self, img):
        r = int(img.shape[0] / 2)
        return img[r:]

    def uncrop(self, img):
        return np.vstack((np.zeros_like(img), img))

    def prepare_input(self, raw_img):
        img_resized = cv2.resize(raw_img, (self.in_width, self.in_height))
        img_blurred = cv2.GaussianBlur(img_resized, (7, 7), 0)
        img_hsv = cv2.cvtColor(img_blurred, cv2.COLOR_BGR2HSV)
        img_scaled = img_hsv.astype(np.float32) / np.array([180., 255., 255.])

        # img_gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)
        # # clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(8, 8))
        # # img_eq = clahe.apply(img_gray)
        # img_blurred = cv2.GaussianBlur(img_gray, (15, 15), 0)
        # img_edges = cv2.Canny(img_blurred, 18, 35)
        # img_edges_scaled = img_edges.astype(np.float32) / 255.
        # img_edges_scaled = img_edges_scaled.reshape(img_edges_scaled.shape + (1,))

        # cv2.imshow("vis", np.hstack([img_blurred, img_edges]))
        # cv2.waitKey(500)

        # return np.concatenate([img_scaled, img_edges_scaled], axis=2)
        return img_scaled

    def load_input(self, fname):
        img = cv2.imread(fname)
        img = self.crop(img)
        return self.prepare_input(img)

    def load_label(self, fname):
        img_label = np.load(fname)
        img_label[img_label == 2] = 1

        onehot_labels = np.zeros(img_label.shape + (self.n_classes,), dtype=np.float32)
        for c in range(self.n_classes):
            onehot_labels[:, :, c][img_label == c] = 1.0

        onehot_labels = self.crop(onehot_labels)
        # cv2.imshow("vis", onehot_labels[..., 1])
        # cv2.waitKey(0)
        return cv2.resize(onehot_labels, (self.in_width, self.in_height))

    def prediction_image_bgr8(self, y):
        colors = [(0, 0, 0), (0, 255, 255), (255, 255, 255), (0, 127, 255)]
        my = np.argmax(y, axis=-1)
        img = np.ndarray((self.in_height, self.in_width, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            mask = (my == i).reshape(img.shape[:2])
            img[mask] = color
        return img

    def prediction_image_mono8(self, y):
        output_img = np.zeros(shape=(self.in_height, self.in_width), dtype=np.uint8)
        argmax_img = np.argmax(y, axis=-1)
        output_img[argmax_img > 0] = 255
        return output_img
