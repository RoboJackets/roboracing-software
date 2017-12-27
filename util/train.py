import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.datasets import mnist
import cv2
import collections
import random

#get our data
data = np.load('data.npy')
np.random.shuffle(data)
n_train = int(len(data) * 0.7)
trainData = data[:n_train]
testData = data[n_train:]

input_shape = (90, 120, 3)
batch_size = 128
epochs = 100
categories = [-0.1, -0.05, 0, 0.05, 0.1]

def defineCategory(steering):
    differences = [abs(steering - category) for category in categories]
    category = np.argmin(differences)
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def format_data(data):
    data2 = np.zeros((len(data),) + input_shape)
    for i in range(len(data)):
        data2[i] = cv2.resize(data[i], (input_shape[1], input_shape[0]))
    return data2

#format our data
xTrain = format_data(trainData[:,0])
xTest = format_data(testData[:,0])
xTrain = xTrain.astype('float32')
xTest  = xTest.astype('float32')
xTrain /= 255
xTest /= 255

#create output bins
yTrain = np.array([defineCategory(steer) for steer in trainData[:,1]])
yTest = np.array([defineCategory(steer) for steer in testData[:,1]])

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

model = Sequential()
# 120x90
model.add(keras.layers.GaussianNoise(0.05, input_shape=input_shape))
model.add(Conv2D(25, (3, 3), activation='relu'))
# 118x88
model.add(Conv2D(50, (3, 3), activation='relu'))
# 116x86
model.add(MaxPooling2D(pool_size=(2, 2)))
# 58x43
model.add(Dropout(0.25))
model.add(Conv2D(100, (3, 3), activation='relu'))
# 56x41
model.add(MaxPooling2D(pool_size=(2, 2)))
# 28x20
model.add(Flatten())
model.add(Dropout(0.25))
model.add(Dense(150, activation='relu'))
model.add(Dropout(0.25))
model.add(Dense(50, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(len(categories), activation='softmax'))

model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

model.fit(xTrain, yTrain,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_data=(xTest, yTest))

score = model.evaluate(xTest, yTest, verbose=0)
print 'Test loss:', score[0]
print 'Test accuracy:', score[1]

model.save('model.h5')
