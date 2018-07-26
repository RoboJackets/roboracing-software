input_shape = (48, 128, 3) # rows, cols, channels


class Config(object):
    def __init__(self, epochs, categories):
        self.epochs = epochs
        self.categories = Config.expand_categories(categories)

    @staticmethod
    def expand_categories(half_cats):
        return [-x for x in half_cats[::-1]] + [0] + half_cats

train_profiles = {
    "circuit": Config(
        epochs=30,
        categories=[0.2, 0.35]
    ),
    "circuit_3cat": Config(
        epochs=20,
        categories=[0.2]
    ),
    "drag": Config(
        epochs=20,
        categories=[0.03]
    )
}
