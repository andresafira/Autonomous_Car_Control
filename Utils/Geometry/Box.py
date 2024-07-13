from Utils.Geometry.Vector import Vector
import numpy as np


class Segment:
    """Method the represents a line segment (used in order to detect collisions)"""
    def __init__(self, P1, P2):
        self.v = P2 - P1
        self.base = P1

    def evaluate(self, t):
        """ Method that calculates the interpolation between P1 and P2 for a given scalar t (t must be between 0 and 1)
        :param t: parametric parameter
        :type t: float
        :return: interpolated point
        :rtype: Vector
        """
        if t < 0.0 or t > 1.0:
            raise Exception("t argument must be a number between 0.0 and 1.0")
        return self.base + self.v * t

    def check_collision(self, other):
        """ Method that checks collision between two segments (in case of no collision, None is returned)
        :param other: other line segment to be checked
        :type other: Segment
        :return: point of intersection if it exists (None otherwise)
        """
        M = np.array([[self.v.x, -other.v.x], [self.v.y, -other.v.y]])
        y = np.array([other.base.x - self.base.x, other.base.y - self.base.y])
        try:
            x = np.linalg.solve(M, y)
        except:
            # Maybe it would be good to add a case for when the objects are parallel and have infinite intersections
            return None
        else:
            if x[0] < 0 or x[0] > 1 or x[1] < 0 or x[1] > 1:
                return None
            return self.evaluate(x[0])


class Box:
    """A set of 4 segements that represents a bounding box. Used to check collision between
    two quadrilateral shapes"""
    def __init__(self, P1, P2, P3, P4):
        self.vertices = [P1, P2, P3, P4]
        self.sides = [Segment(P1, P2), Segment(P2, P3), Segment(P3, P4), Segment(P4, P1)]

    def check_collision_segment(self, segment):
        """ Method that checks collision between the box and a segment
        :param segment: the segment which is to be checked the collision
        :type segment: Segment
        :return: list of collision points if they exist (None otherwise)
        """
        results = []
        for side in self.sides:
            point = segment.check_collision(side)
            if point is not None:
                results.append(point)
        if len(results) == 0:
            return None
        return results

    def check_collision_box(self, box):
        """ Method that checks if two boxes collide
        :param box: other box to be checked
        :type box: Box
        :return: if there was or not collision
        :rtype: bool
        """
        for this_side in self.sides:
            for other_side in box.sides:
                collision = this_side.check_collision(other_side)
                if collision is not None:
                    return True
        return False

    def check_collision(self, objects):
        """More general function that performs the collision checking of a list of
        objects, that can be segements of other boxes"""
        if type(objects) is Segment:
            return self.check_collision_segment(objects) is not None
        if type(objects) is Box:
            return self.check_collision_box(objects)
        if type(objects) is not list and type(objects) is not tuple:
            raise TypeError('Invalid input type')
        for obj in objects:
            if (type(obj) is Segment) and (self.check_collision_segment(obj) is not None):
                return True
            elif (type(obj) is Box) and (self.check_collision_box(obj)):
                return True
        return False
