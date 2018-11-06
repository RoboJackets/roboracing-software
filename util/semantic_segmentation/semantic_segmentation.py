import os
import sys
import glob
import random
import time

import cv2
import keras
import numpy as np

osp = os.path

squeeze_height = 54
squeeze_width = 96
in_height = squeeze_height * 4
in_width = squeeze_width * 4
n_channels = 3
n_categories = 3

batch_size = 10


# significant credit to https://github.com/divamgupta/image-segmentation-keras
def make_model():
    inputs = keras.layers.Input((in_height, in_width, n_channels))

    conv1 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(inputs)
    conv1 = keras.layers.Dropout(0.2)(conv1)
    conv1 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(conv1)

    pool1 = keras.layers.MaxPooling2D(pool_size=2)(conv1)

    conv2 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(pool1)
    conv2 = keras.layers.Dropout(0.2)(conv2)
    conv2 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(conv2)

    pool2 = keras.layers.MaxPooling2D(pool_size=2)(conv2)

    conv3 = keras.layers.Conv2D(64, 3, activation='relu', padding='same')(pool2)
    conv3 = keras.layers.Dropout(0.2)(conv3)
    conv3 = keras.layers.Conv2D(64, 3, activation='relu', padding='same')(conv3)

    up1 = keras.layers.UpSampling2D(size=2)(conv3)
    up1 = keras.layers.Concatenate(axis=3)([up1, conv2])

    conv4 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(up1)
    conv4 = keras.layers.Dropout(0.2)(conv4)
    conv4 = keras.layers.Conv2D(32, 3, activation='relu', padding='same')(conv4)

    up2 = keras.layers.UpSampling2D(size=2)(conv4)
    up2 = keras.layers.Concatenate(axis=3)([up2, conv1])

    conv5 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(up2)
    conv5 = keras.layers.Dropout(0.2)(conv5)
    conv5 = keras.layers.Conv2D(16, 3, activation='relu', padding='same')(conv5)

    conv6 = keras.layers.Conv2D(n_categories, 1, padding='same')(conv5)
    conv6 = keras.layers.Reshape((in_height * in_width, n_categories))(conv6)
    conv7 = keras.layers.Activation('softmax')(conv6)

    model = keras.Model(inputs=inputs, outputs=conv7)
    return model


def load_data(input_file, label_file):
    out = [None, None]

    img = cv2.imread(input_file)
    img = img.astype(np.float32) / 255.0
    out[0] = cv2.resize(img, (in_width, in_height))

    img = cv2.imread(label_file)
    img = img[:, :, 0]
    label_img = np.zeros(img.shape[:2] + (n_categories,), dtype=np.uint8)
    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            chan = img[r, c]
            label_img[r, c, chan] = 1

    label_img = cv2.resize(label_img, (in_width, in_height))
    out[1] = np.zeros((in_height * in_width, n_categories), dtype=np.float32)
    for r in range(label_img.shape[0]):
        for c in range(label_img.shape[1]):
            i = r * label_img.shape[1] + c
            out[1][i, :] = label_img[r, c, :]

    return out


def data_gen(input_files, label_files):
    b = 0
    while b + batch_size <= len(input_files):
        inputs = np.zeros((batch_size, in_height, in_width, n_channels), dtype=np.float32)
        labels = np.zeros((batch_size, in_height * in_width, n_categories), dtype=np.float32)

        for n in range(batch_size):
            inputs[n], labels[n] = load_data(input_files[b + n], label_files[b + n])

        b += batch_size
        yield inputs, labels


def data_gen_wrapper(input_files, label_files, epochs):
    for epoch in range(epochs):
        for x in data_gen(input_files, label_files):
            yield x


def output_to_image(y):
    colors = [(0,0,0), (255,0,0), (0,255,0)]
    img = np.ndarray((in_height, in_width, 3), dtype=np.uint8)
    for r in range(in_height):
        for c in range(in_width):
            categories = y[r * in_width + c]
            i = np.argmax(categories)
            # print(categories)
            img[r, c] = colors[i]
    return img


def main():
    data_dir = osp.join(osp.split(__file__)[0], "data")

    input_files = sorted(glob.glob(osp.join(data_dir, "test_img_*.png")))
    label_files = sorted(glob.glob(osp.join(data_dir, "test_label_*.png")))

    files = list(zip(input_files, label_files))
    random.shuffle(files)
    split = int(len(input_files) * 0.75)
    training_files = files[:split]
    validate_files = files[split:]

    training_input_files, training_label_files = zip(*training_files)
    validate_input_files, validate_label_files = zip(*validate_files)

    try:
        model = keras.models.load_model("model.h5")
        print "loaded model model.h5 from disk"
    except:
        model = make_model()
        model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
        print "no model found on disk, created a new one"
        model.summary()

    n_epochs = 1
    if "-e" in sys.argv:
        i = sys.argv.index("-e")
        n_epochs = int(sys.argv[i + 1])

    if not "--no-train" in sys.argv:
        model.fit_generator(data_gen_wrapper(training_input_files, training_label_files, n_epochs),
                            steps_per_epoch=len(training_files) // batch_size, epochs=n_epochs,
                            verbose=1, max_queue_size=5)

        keras.models.save_model(model, "model.h5")
        print "saved model.h5 to disk"

    loss, accuracy = model.evaluate_generator(data_gen(validate_input_files, validate_label_files),
                                              steps=len(validate_files) // batch_size, verbose=1)

    print "loss", loss, "accuracy", accuracy

    scores = []
    pred_times = []
    for fx, fy in zip(validate_input_files, validate_label_files):
        x, y_ = load_data(fx, fy)

        t_pred_0 = time.time()
        y = model.predict(x[None])[0]
        t_pred = time.time() - t_pred_0
        pred_times.append(t_pred)

        right = total = 0
        for a, b in zip(y, y_):
            ma = np.argmax(a)
            mb = np.argmax(b)
            if ma > 0 or mb > 0:
                total += 1
                if ma == mb:
                    right += 1

        scores.append(float(right) / total)

        cv2.imshow("input", (x * 255).astype(np.uint8))
        cv2.imshow("ground truth", output_to_image(y_))
        cv2.imshow("output", output_to_image(y))
        cv2.waitKey(1)

    print "avg IOU =", sum(scores) / len(scores)
    print "avg prediction time", sum(pred_times) / len(pred_times)


if __name__ == '__main__':
    main()
