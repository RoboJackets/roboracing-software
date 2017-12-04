import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras import backend

def defineCategory(steering):
    categories = [-0.25, -0.1, 0, 0.1, 0.25]
    differences = [abs(steering - category) for category in categories]
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

#get our data
data = np.load('data.npy')
print type(data[0])
trainData = data[:1500]
testData = data[1500:]
print backend.image_data_format()

#format our data
xTrain  = trainData[:,0]
xTrain.reshape(1500, 240, 320, 3)
xTest = testData[:,0]
xTrain = xTrain.astype('float32')
xTest  = xTest.astype('float32')
xTrain /= 255
xTest /= 255

#create output bins
yTrain = trainData[:,1]
yTest = testData[:,1]
yTrain = [defineCategory(steer) for steer in yTrain]
yTest = [defineCategory(steer) for steer in yTest]


print "x train " + str(xTrain.shape)
print "y train " + str(len(yTrain)) + " " + str(len(yTrain[0]))
print "x test " + str(xTest.shape)
print "y test ", len(yTest), len(yTrain[0])
model = Sequential()

