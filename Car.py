from Utils.Geometry.Vector import Vector
from Utils.Geometry.Position import Position
from Utils.Geometry.Box import Segment, Box
from Utils.General import sgn, interpolate_cycle
from constants import CAR_ACCELERATION, CAR_MAX_SPEED, CAR_ANGLE_STEP, CAR_WIDTH, CAR_HEIGHT
from constants import CAR_START_LEFT, CAR_START_RIGHT, CAR_MASS, WIND_B
from constants import MAX_STEERING_WHEEL_ANGLE, INERTIA_PARAMETER_WHEEL 
from constants import SAMPLE_TIME, eps
from math import cos, sin, pi, fabs, sqrt, tan
from Control import PFController


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

    def set_controllers(self, speed_controller, position_controller):
        self.position_controller = position_controller
        self.speed_controller = speed_controller

    def accelerate(self):
        self.speed += CAR_ACCELERATION * SAMPLE_TIME
        self.speed = min(self.speed, CAR_MAX_SPEED)

    def brake(self):
        self.speed -= CAR_ACCELERATION * SAMPLE_TIME
        self.speed = max(self.speed, -CAR_MAX_SPEED)

    def turn_right(self):
        self.steering_wheel_angle += CAR_ANGLE_STEP
        self.steering_wheel_angle = min(self.steering_wheel_angle, MAX_STEERING_WHEEL_ANGLE)

    def turn_left(self):
        self.steering_wheel_angle -= CAR_ANGLE_STEP
        self.steering_wheel_angle = max(self.steering_wheel_angle, -MAX_STEERING_WHEEL_ANGLE)

    def apply_command(self, vr, xr):
        v = max(self.speed, CAR_ACCELERATION*SAMPLE_TIME)
        if self.position_controller.name == "PV":
            phi = self.position_controller.control(xr, self.position.location.x, self.position.rotation, v, CAR_HEIGHT)
        else:
            phi = self.position_controller.control(xr, self.position.location.x)
        self.steering_wheel_angle = phi

        f = self.speed_controller.control(vr, self.speed)
        self.speed += f/CAR_MASS * SAMPLE_TIME
        self.speed = min(max(self.speed, -CAR_MAX_SPEED), CAR_MAX_SPEED)

    def move(self):
        self.position.location += Vector(sin(self.position.rotation), cos(self.position.rotation)) * self.speed * SAMPLE_TIME
        self.position.rotation += self.speed * tan(self.steering_wheel_angle) / CAR_HEIGHT * SAMPLE_TIME

        if fabs(self.steering_wheel_angle) < 2 * eps * pi:
            self.steering_wheel_angle = 0

        self.speed -= self.speed * WIND_B / CAR_MASS * SAMPLE_TIME
        if fabs(self.speed) < eps:
            self.speed = 0

    def update(self, world_objects=None):
        if not self.alive:
            return
        if self.DUMMY:
            self.position.location.y += self.speed*SAMPLE_TIME
            self.bounding_box = self.unravel_box()
            return
        self.move()
        if world_objects is not None:
            self.bounding_box = self.unravel_box()
            if self.bounding_box.check_collision(world_objects):
                self.alive = False

    def unravel_box(self):
        theta = self.position.rotation
        P1 = self.position.location + Vector(sin(theta) * CAR_HEIGHT / 2 - cos(theta) * CAR_WIDTH / 2,
                                             cos(theta) * CAR_HEIGHT / 2 + sin(theta) * CAR_WIDTH / 2)
        P2 = P1 + Vector(cos(theta), -sin(theta)) * CAR_WIDTH
        P3 = P2 + Vector(-sin(theta), -cos(theta)) * CAR_HEIGHT
        P4 = P3 + Vector(-cos(theta), sin(theta)) * CAR_WIDTH
        return Box(P1, P2, P3, P4)
