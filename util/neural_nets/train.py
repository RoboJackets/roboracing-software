import os, sys
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, GaussianNoise
from keras.datasets import mnist
import cv2
import collections
from example_set import ExampleSetArchiver, ExampleSet
import random


n_examples_to_load = 6000 # if the number of training examples is below this, load another data archive
input_shape = (90, 120, 3) # rows, cols, channels
batch_size = 128
epochs = 15
categories = [-0.1, -0.05, 0, 0.05, 0.1]


def defineCategory(steering):
    differences = [abs(steering - category) for category in categories]
    category = np.argmin(differences)
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def format_data(data):
    data2 = np.zeros((len(data),) + input_shape, dtype='float16')
    for i in range(len(data)):
        data2[i] = cv2.resize(data[i], (input_shape[1], input_shape[0]))
    return data2


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "Usage: python train.py [data folder] [model output file]"
        sys.exit(1)

    model = Sequential()
    # 120x90
    model.add(GaussianNoise(0.05, input_shape=input_shape))
    model.add(Conv2D(30, (3, 3), activation='relu'))
    # 118x88
    model.add(MaxPooling2D((2,2)))
    # 59x44
    model.add(Conv2D(50, (3, 3), activation='relu'))
    # 57x42
    model.add(MaxPooling2D((4, 4)))
    # 14x10

    model.add(Flatten())
    model.add(Dropout(0.3))
    model.add(Dense(120, activation='relu'))
    model.add(Dropout(0.1))
    model.add(Dense(50, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(len(categories), activation='softmax'))

    model.compile(loss=keras.losses.categorical_crossentropy,
                  optimizer=keras.optimizers.Adadelta(),
                  metrics=['accuracy'])


    archiver = ExampleSetArchiver(sys.argv[1])
    for epoch in range(epochs):
        print "epoch", epoch+1

        data = ExampleSet()
        indices = range(len(archiver.sets))
        random.shuffle(indices)
        for i in indices:
            if len(data.train) >= n_examples_to_load:
                break
            data.add(archiver.get(i))

        #format our data
        xTrain = format_data([ex.get_image() for ex in data.train])
        xTest = format_data([ex.get_image() for ex in data.test])
        xTrain = xTrain.astype('float16')
        xTest  = xTest.astype('float16')
        xTrain /= 255
        xTest /= 255

        #create output bins
        yTrain = np.array([defineCategory(ex.angle) for ex in data.train])
        yTest = np.array([defineCategory(ex.angle) for ex in data.test])

        # cnt = collections.Counter()
        # for x,y in zip(xTrain,yTrain):
        #     i = np.argmax(y)
        #     cnt[i] += 1
        # print "training label counts:", cnt

        # cnt = collections.Counter()
        # for x,y in zip(xTest,yTest):
        #     i = np.argmax(y)
        #     cnt[i] += 1
        # print "testing label counts:", cnt

        model.fit(xTrain, yTrain,
                  batch_size=batch_size,
                  epochs=1,
                  verbose=1,
                  validation_data=(xTest, yTest))

        # score = model.evaluate(xTest, yTest, verbose=0)
        # print 'Test loss:', score[0]
        # print 'Test accuracy:', score[1]

    model.save(sys.argv[2])
