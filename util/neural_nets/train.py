import os, sys
import math
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D, \
                         GaussianNoise, BatchNormalization
import cv2
import collections
import random
import time
from example_set import ExampleSet
from params import input_shape, train_profiles


n_examples_to_load = 8000 # if the number of training examples is below this, load more data
batch_size = 32


def defineCategory(steering):
    differences = [abs(steering - category) for category in categories]
    category = np.argmin(differences)
    oneHot = [1 if i == category else 0 for i in range(len(categories))]
    return oneHot

def format_inputs(examples):
    data2 = np.zeros((len(examples),) + input_shape, dtype='float32')
    for i, ex in enumerate(examples):
        data2[i] = ex.get_image()
    data2 /= 255.0
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
    if len(sys.argv) < 4:
        print "Usage: python train.py [data directory] [model output file] {%s}" \
                % ', '.join(train_profiles.keys())
        sys.exit(1)

    startTime = time.time()

    model_path = sys.argv[2]

    config = train_profiles[sys.argv[3]]
    epochs = config.epochs
    categories = config.categories

    model = make_model()
    model.summary()

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

    def batch_generator(isValidation = False):
        gen_epochs = 1 if isValidation else epochs
        for epoch in range(gen_epochs):
            exampleSetFiles = list(exampleSetFiles_const)
            random.shuffle(exampleSetFiles)

            while len(exampleSetFiles) > 0:
                D = []
                while len(exampleSetFiles) > 0 and len(D) < n_examples_to_load:
                    data = ExampleSet.load(os.path.join(exampleSetDir, exampleSetFiles.pop()))
                    D += data.test if isValidation else data.train

                if not isValidation: random.shuffle(D)

                X = format_inputs(D)

                #create output bins
                labels = np.array([defineCategory(ex.angle) for ex in D])

                if not isValidation:
                    for i in range(len(X)):
                        if random.random() < 0.4: # 40% of images are flipped
                            X[i] = cv2.flip(X[i], 1)
                            labels[i] = labels[i][::-1]

                for i in range(0, len(X), batch_size):
                    xs = X[i: i + batch_size]
                    ys = labels[i: i + batch_size]
                    yield (xs, ys)

    try:
        n_minibatches = int(math.ceil(float(n_training_examples) / batch_size))
        model.fit_generator(batch_generator(),
                            steps_per_epoch=n_minibatches,
                            epochs=epochs,
                            verbose=1)
        print "elapsed time:", time.time() - startTime

        n_minibatches = int(math.ceil(float(n_test_examples) / batch_size))
        loss, acc = model.evaluate_generator(batch_generator(True), steps=n_minibatches)
        print "validation loss:", loss, "| validation accuracy:", acc

    finally:
        model.save(model_path)
        print "\nsaved model to", model_path
