from Utils.Geometry.Vector import Vector
from Utils.Geometry.Position import Position
from Utils.Geometry.Box import Segment, Box
from Utils.General import sgn, interpolate_cycle
from simulation_constants import CAR_ACCELERATION, CAR_MAX_SPEED, CAR_ANGLE_STEP, CAR_WIDTH, CAR_HEIGHT
from simulation_constants import CAR_START_LEFT, CAR_START_RIGHT
from simulation_constants import MAX_STEERING_WHEEL_ANGLE, INERTIA_PARAMETER_WHEEL, INERTIA_PARAMETER_SPEED
from simulation_constants import SAMPLE_TIME, eps
from math import cos, sin, pi, fabs, sqrt

class ControlSystem:
    def __init__(self, kp, ki, kd, sampling_time):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sampling_time

    def control(self, error):
        pass



class Car:
    def __init__(self, initial_position, initial_speed=0, DUMMY=False):
        self.DUMMY = DUMMY
        self.position = initial_position
        self.speed = initial_speed
        self.bounding_box = self.unravel_box()
        self.keys_history = []
        self.alive = True
        if not self.DUMMY:
            self.steering_wheel_angle = 0

    def accelerate(self):
        self.speed += CAR_ACCELERATION
        self.speed = min(self.speed, CAR_MAX_SPEED)

    def brake(self):
        self.speed -= CAR_ACCELERATION
        self.speed = max(self.speed, -CAR_MAX_SPEED)

    def turn_right(self):
        self.steering_wheel_angle += CAR_ANGLE_STEP
        self.steering_wheel_angle = min(self.steering_wheel_angle, MAX_STEERING_WHEEL_ANGLE)

    def turn_left(self):
        self.steering_wheel_angle -= CAR_ANGLE_STEP
        self.steering_wheel_angle = max(self.steering_wheel_angle, -MAX_STEERING_WHEEL_ANGLE)

    def move(self):
        speed_translation = self.speed * cos(self.steering_wheel_angle)
        speed_rotation = self.speed * sin(self.steering_wheel_angle)
        self.position.location += Vector(-sin(self.position.rotation), cos(self.position.rotation)) * speed_translation
        increment = Vector(cos(self.position.rotation), sin(self.position.rotation))*(speed_rotation/2)
        self.position.location += increment
        self.position.rotation -= speed_rotation / CAR_WIDTH
        self.steering_wheel_angle -= sgn(self.steering_wheel_angle) * INERTIA_PARAMETER_WHEEL
        if fabs(self.steering_wheel_angle) < 3 * eps * pi:
            self.steering_wheel_angle = 0
        self.speed -= sgn(self.speed) * INERTIA_PARAMETER_SPEED
        if fabs(self.speed) < eps:
            self.speed = 0

    def update(self, world_objects=None):
        if not self.alive:
            return
        if self.DUMMY:
            self.position.location.y += self.speed
            self.bounding_box = self.unravel_box()
            return
        self.move()
        if world_objects is not None:
            self.bounding_box = self.unravel_box()
            if self.bounding_box.check_collision(world_objects):
                self.alive = False

    def unravel_box(self):
        theta = self.position.rotation
        P1 = self.position.location + Vector(-sin(theta) * CAR_HEIGHT / 2 - cos(theta) * CAR_WIDTH / 2,
                                             cos(theta) * CAR_HEIGHT / 2 - sin(theta) * CAR_WIDTH / 2)
        P2 = P1 + Vector(cos(theta), sin(theta)) * CAR_WIDTH
        P3 = P2 + Vector(sin(theta), -cos(theta)) * CAR_HEIGHT
        P4 = P1 + Vector(sin(theta), -cos(theta)) * CAR_HEIGHT
        return Box(P1, P2, P3, P4)
