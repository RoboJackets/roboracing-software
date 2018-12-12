import cPickle as pickle
import lz4
import numpy as np


class Example:
    def __init__(self, img, angle):
        self.img_buf = lz4.block.compress(img)
        self.img_shape = img.shape
        self.angle = angle

    def get_image(self):
        return np.frombuffer(lz4.block.decompress(self.img_buf), dtype='uint8').reshape(self.img_shape)


class ExampleSet:
    def __init__(self):
        self.train = []
        self.test = []

    def add(self, other):
        self.train += other.train
        self.test += other.test

    def save(self, fname):
        with open(fname, 'w') as f:
            f.write(lz4.block.compress(pickle.dumps(self)))

    @staticmethod
    def load(fname):
        with open(fname, 'r') as f:
            obj = pickle.loads(lz4.block.decompress(f.read()))
        return obj
