import os, sys
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, GaussianNoise, BatchNormalization
# from keras.datasets import mnist
# from keras.preprocessing.image import ImageDataGenerator
import cv2
import collections
import random
import time
from example_set import ExampleSet
from params import input_shape


n_examples_to_load = 8000 # if the number of training examples is below this, load more data
batch_size = 32
epochs = 20
categories = [-0.2, -0.05, 0, 0.05, 0.2]


def defineCategory(steering):
    differences = [abs(steering - category) for category in categories]
    category = np.argmin(differences)
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def format_inputs(examples):
    data2 = np.zeros((len(examples),) + input_shape, dtype='float32')
    for i, ex in enumerate(examples):
        data2[i] = ex.get_image()
    return data2

def make_model():
    model = Sequential()
    # 128 x 48
    model.add(GaussianNoise(0.05, input_shape=input_shape))
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((4, 4)))
    # 32 x 12
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))
    # 16 x 6

    model.add(Flatten())
    model.add(Dropout(0.25))
    model.add(Dense(128, activation='relu'))
    model.add(Dense(32, activation='relu'))
    model.add(Dropout(0.35))
    model.add(Dense(len(categories), activation='softmax'))

    model.compile(loss=keras.losses.categorical_crossentropy,
                  optimizer=keras.optimizers.Adadelta(),
                  metrics=['accuracy'])
    return model


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "Usage: python train.py [data directory] [model output file]"
        sys.exit(1)

    startTime = time.time()

    name = sys.argv[2]

    model = make_model()

    exampleSetDir = sys.argv[1]
    exampleSetFiles_const = tuple(f for f in os.listdir(exampleSetDir) if '.pkl.lz4' in f)
    n_training_examples = 0
    n_test_examples = 0
    cnt = collections.Counter()
    for f in exampleSetFiles_const:
        data = ExampleSet.load(os.path.join(exampleSetDir, f))
        n_training_examples += len(data.train)
        n_test_examples += len(data.test)
        for ex in data.train:
            i = np.argmax(defineCategory(ex.angle))
            cnt[i] += 1

    print "total training examples:", n_training_examples
    print "training label counts:", cnt

    def batch_generator(isTraining):
        for epoch in range(epochs):
            exampleSetFiles = list(exampleSetFiles_const)
            random.shuffle(exampleSetFiles)

            while len(exampleSetFiles) > 0:
                D = []
                while len(exampleSetFiles) > 0 and len(D) < n_examples_to_load:
                    data = ExampleSet.load(os.path.join(exampleSetDir, exampleSetFiles.pop()))
                    D += data.train if isTraining else data.test

                if isTraining: random.shuffle(D)

                #format our data
                X = format_inputs(D)
                X /= 255.0

                #create output bins
                labels = np.array([defineCategory(ex.angle) for ex in D])

                if isTraining:
                    for i in range(len(X)):
                        if random.random() < 0.4: # 40% of images are flipped
                            X[i] = cv2.flip(X[i], 1)
                            labels[i] = labels[i][::-1]
                        # print(y[i])

                for i in range(0, len(X), batch_size):
                    xs = X[i: i + batch_size]
                    ys = labels[i: i + batch_size]
                    yield (xs, ys)

    """
    keras.preprocessing.image.ImageDataGenerator(featurewise_center=False, samplewise_center=False,
        featurewise_std_normalization=False, samplewise_std_normalization=False, zca_whitening=False,
        zca_epsilon=1e-06, rotation_range=0.0, width_shift_range=0.0, height_shift_range=0.0,
        brightness_range=None, shear_range=0.0, zoom_range=0.0, channel_shift_range=0.0, fill_mode='nearest',
        cval=0.0, horizontal_flip=False, vertical_flip=False, rescale=None, preprocessing_function=None,
        data_format=None, validation_split=0.0)
    """
    # augmented_data_generator = ImageDataGenerator()

        # augmented_data_generator.fit(xTrain)
        # model.fit_generator(augmented_data_generator.flow(xTrain, yTrain, batch_size = batch_size),
        #     epochs = 1,
        #     verbose = 1,
        #     validation_data=(xTest, yTest))


    model.fit_generator(batch_generator(True), steps_per_epoch=n_training_examples//batch_size + 1,
                        epochs=epochs, verbose=1)

    model.save(name)

    loss, acc = model.evaluate_generator(batch_generator(False), steps=n_test_examples//batch_size + 1)
    print "validation loss:", loss, "| validation accuracy:", acc

    print "elapsed time:", time.time() - startTime
