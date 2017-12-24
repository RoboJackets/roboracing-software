import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras import backend
from keras.datasets import mnist

def defineCategory(steering):
    categories = [-0.25, -0.1, 0, 0.1, 0.25]
    differences = [abs(steering - category) for category in categories]
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def flatten(data):
    dataFlattened = np.zeros((len(data), 240, 320, 3))
    for i in range(len(data)):
        dataFlattened[i] = data[i]
    return dataFlattened

#get our data
data = np.load('data.npy')
print type(data[0])
trainData = data[:1500]
testData = data[1500:]
print backend.image_data_format()
input_shape = (240, 320, 3)
num_classes = 5
batch_size = 150
epochs = 12

#format our data
xTrain = flatten(trainData[:,0])
#xTrain.reshape(1500, 240, 320, 3)

print xTrain.shape
xTest = flatten(testData[:,0])
xTrain = xTrain.astype('float32')
xTest  = xTest.astype('float32')
xTrain /= 255
xTest /= 255

#create output bins
yTrain = trainData[:,1]
yTest = testData[:,1]
yTrain = np.array([defineCategory(steer) for steer in yTrain])
yTest = np.array([defineCategory(steer) for steer in yTest])

model = Sequential()
model.add(Conv2D(32, kernel_size=(3, 3),
                 activation='relu',
                 input_shape=input_shape))
model.add(Conv2D(64, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))
model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(num_classes, activation='softmax'))

model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

model.fit(xTrain, yTrain,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_data=(xTest, yTest))
score = model.evaluate(xTest, yTest, verbose=0)
print('Test loss:', score[0])
print('Test accuracy:', score[1])
