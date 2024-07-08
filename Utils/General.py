from math import pi

# constant for float comparison
eps = 1e-3


def interpolate_cycle(num_steps):
    """ Function that returns interpolated points in a cycle for a given number of interpolated
        points (the interpolated points contain the begin and end positions)
    :param num_steps: number of interpolated points
    :type num_steps: int
    :return: list with the interpolated points
    :rtype: list
    """
    return [2 * pi / num_steps * i for i in range(num_steps)]


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
