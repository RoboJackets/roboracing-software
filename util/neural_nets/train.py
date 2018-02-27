import os, sys
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, GaussianNoise
from keras.datasets import mnist
import cv2
import collections
import random
from example_set import ExampleSet


n_examples_to_load = 4096 # if the number of training examples is below this, load more data
input_shape = (96, 128, 3) # rows, cols, channels
batch_size = 32
epochs = 100
categories = [-0.17, -0.05, 0, 0.05, 0.17]


def defineCategory(steering):
    differences = [abs(steering - category) for category in categories]
    category = np.argmin(differences)
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def format_data(data):
    data2 = np.zeros((len(data),) + input_shape, dtype='float32')
    for i in range(len(data)):
        data2[i] = cv2.resize(data[i], (input_shape[1], input_shape[0]))
    return data2


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "Usage: python train.py [data directory] [model output file]"
        sys.exit(1)

    name = sys.argv[2]

    model = Sequential()
    # 128 x 96
    model.add(GaussianNoise(0.05, input_shape=input_shape))
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))
    # 64 x 48
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))
    # 32 x 24
    model.add(Conv2D(96, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(96, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2)))
    # 16 x 12

    model.add(Flatten())
    model.add(Dropout(0.1))
    model.add(Dense(200, activation='relu'))
    model.add(Dense(100, activation='relu'))
    model.add(Dense(50, activation='relu'))
    model.add(Dropout(0.4))
    model.add(Dense(len(categories), activation='softmax'))

    model.compile(loss=keras.losses.categorical_crossentropy,
                  optimizer=keras.optimizers.Adadelta(),
                  metrics=['accuracy'])


    for epoch in range(epochs):
        print "\nbatch", epoch+1

        data = ExampleSet()
        sets = [f for f in os.listdir(sys.argv[1]) if '.pkl.lz4' in f]
        indices = range(len(sets))
        random.shuffle(indices)
        for i in indices:
            if len(data.train) >= n_examples_to_load:
                break
            data.add(ExampleSet.load(os.path.join(sys.argv[1], sets[i])))

        random.shuffle(data.train)

        #format our data
        xTrain = format_data([ex.get_image() for ex in data.train])
        xTest = format_data([ex.get_image() for ex in data.test])
        xTrain = xTrain.astype('float32')
        xTest  = xTest.astype('float32')
        xTrain /= 255
        xTest /= 255

        #create output bins
        yTrain = np.array([defineCategory(ex.angle) for ex in data.train])
        yTest = np.array([defineCategory(ex.angle) for ex in data.test])

        for i in range(len(xTrain)):
            if random.random() < 0.4: # 60-40
                xTrain[i] = cv2.flip(xTrain[i], 1)
                yTrain[i] = yTrain[i][::-1]
            # print(yTrain[i])

        cnt = collections.Counter()
        for x,y in zip(xTrain,yTrain):
            i = np.argmax(y)
            cnt[i] += 1
        print "training label counts:", cnt

        cnt = collections.Counter()
        for x,y in zip(xTest,yTest):
            i = np.argmax(y)
            cnt[i] += 1
        print "testing label counts:", cnt

        model.fit(xTrain, yTrain,
                  batch_size=batch_size,
                  epochs=1,
                  verbose=1,
                  validation_data=(xTest, yTest))

        tmp_name = os.path.join(os.path.dirname(__file__), "model_ckpt.h5")
        model.save(tmp_name)

    model.save(name)
