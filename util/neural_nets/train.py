import os, sys
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, GaussianNoise, BatchNormalization
from keras.datasets import mnist
import cv2
import collections
import random
import time
from example_set import ExampleSet
from params import input_shape


n_examples_to_load = 8000 # if the number of training examples is below this, load more data
batch_size = 64
epochs = 10
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


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "Usage: python train.py [data directory] [model output file]"
        sys.exit(1)

    startTime = time.time()

    name = sys.argv[2]

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

    exampleSetDir = sys.argv[1]
    exampleSetFiles = [f for f in os.listdir(exampleSetDir) if '.pkl.lz4' in f]
    exampleSetFiles = exampleSetFiles * epochs  # duplicate data [epochs] times
    random.shuffle(exampleSetFiles)

    while len(exampleSetFiles) > 0:
        print "\n", len(exampleSetFiles), "training files remaining"

        data = ExampleSet()
        while len(exampleSetFiles) > 0 and len(data.train) < n_examples_to_load:
            data.add(ExampleSet.load(os.path.join(exampleSetDir, exampleSetFiles.pop())))
        random.shuffle(data.train)

        #format our data
        xTrain = format_inputs(data.train)
        xTest = format_inputs(data.test)
        xTrain /= 255.0
        xTest /= 255.0

        #create output bins
        yTrain = np.array([defineCategory(ex.angle) for ex in data.train])
        yTest = np.array([defineCategory(ex.angle) for ex in data.test])

        for i in range(len(xTrain)):
            if random.random() < 0.4: # 40% of images are flipped
                xTrain[i] = cv2.flip(xTrain[i], 1)
                yTrain[i] = yTrain[i][::-1]
            # print(yTrain[i])

        cnt = collections.Counter()
        for y in yTrain:
            i = np.argmax(y)
            cnt[i] += 1
        print "training label counts:", cnt

        model.fit(xTrain, yTrain,
                  batch_size=batch_size,
                  epochs=1,
                  verbose=1,
                  validation_data=(xTest, yTest))

        model.save(name)

    print "elapsed time:", time.time() - startTime

    # print "final validation over all data:"
    # for fname in os.listdir(exampleSetDir):
    #     if ".pkl.lz4" in fname:
