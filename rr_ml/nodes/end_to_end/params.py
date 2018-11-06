input_shape = (48, 128, 3)  # rows, cols, channels


def expand_categories(half_cats):
    return [-x for x in half_cats[::-1]] + [0] + half_cats
