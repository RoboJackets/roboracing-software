import cPickle as pickle
from StringIO import StringIO
import gzip, lz4
import os
import numpy as np


class Example:
    def __init__(self, img, angle):
        self.img_buf = lz4.compress(img)
        self.img_shape = img.shape
        self.angle = angle

    def get_image(self):
        return np.frombuffer(lz4.decompress(self.img_buf), dtype='uint8').reshape(self.img_shape)


class ExampleSet:
    def __init__(self):
        self.train = []
        self.test = []

    def add(self, other):
        self.train += other.train
        self.test += other.test

    def save(self, fname):
        with gzip.GzipFile(fname, 'w') as f:
            pickle.dump(self, f)

    @staticmethod
    def load(fname):
        with gzip.GzipFile(fname, 'r') as f:
            obj = pickle.load(f)
        return obj


class ExampleSetArchiver:
    def __init__(self, directory):
        self.sets = []
        self.dir = directory

        if not os.path.exists(self.dir):
            os.makedirs(self.dir)

        for fname in os.listdir(self.dir):
            if '.pkl.gz' in fname:
                self.sets.append(fname)

    def store(self, example_set, examples_per_file):
        sets = []
        while len(example_set.train) > examples_per_file * 1.5:
            tmp = ExampleSet()
            tmp.train = example_set.train[:examples_per_file]
            tmp.test = example_set.test[:examples_per_file]
            sets.append(tmp)
            example_set.train = example_set.train[examples_per_file:]
            example_set.test = example_set.test[examples_per_file:]
        sets.append(example_set)

        fname = lambda i: "data%d.pkl.gz" % i

        for example_set in sets:
            existing = os.listdir(self.dir)
            for i in range(len(self.sets) + 1):
                if fname(i) not in existing:
                    print "saving %s..." % fname(i)
                    example_set.save(os.path.join(self.dir, fname(i)))
                    self.sets.append(fname(i))
                    break

    def get(self, i):
        return ExampleSet.load(os.path.join(self.dir, self.sets[i]))
