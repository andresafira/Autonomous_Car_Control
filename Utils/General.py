from math import pi

# constant for float comparison
eps = 1e-3


def clip(number, limit):
    """ Function that clips a number under a certain limit (works similarly to a mod function)
    :param number: number to be clipped
    :param limit: mod like number to be used as a clip bound
    :return: the clipped value
    """
    times = number // limit
    return number - times * limit


def sgn(number):
    """ Sign function (it uses a eps value for comparison between floats)
    :param number: argument
    :return: sign of the number
    """
    if number < -eps:
        return -1
    if number > eps:
        return 1
    return 0
