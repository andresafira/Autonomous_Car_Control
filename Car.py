from Utils.Geometry.Vector import Vector
from Utils.Geometry.Position import Position
from Utils.Geometry.Box import Segment, Box
from Utils.General import sgn
from constants import CAR_ACCELERATION, CAR_MAX_SPEED, CAR_ANGLE_STEP, CAR_WIDTH, CAR_HEIGHT
from constants import CAR_START_LEFT, CAR_START_RIGHT, CAR_MASS, WIND_B
from constants import MAX_STEERING_WHEEL_ANGLE, INERTIA_PARAMETER_WHEEL 
from constants import SAMPLE_TIME, eps
from math import cos, sin, pi, fabs, sqrt, tan
from Control import PFController, FullPIDController


class Car:
    """Car implementation class"""
    def __init__(self, initial_position: Position, initial_speed: float = 0.0, DUMMY: bool = False):
        """Initialization method. It generates a car in a
            given position with a given initial speed.
            If DUMMY is passed as True, then the car will only
            move with the given constant speed"""

        self.DUMMY: bool = DUMMY

        self.position: Position = initial_position
        self.speed: float = initial_speed

        self.bounding_box: Box = self.unravel_box()

        self.alive: bool = True

        # It tells if the car will be controlled by the user
        # or by the specified control system
        self.playable: bool = True

        if not self.DUMMY:
            self.steering_wheel_angle: float = 0

    def set_controllers(self, speed_controller: PFController,
                              position_controller: FullPIDController):
        self.speed_controller = speed_controller
        self.position_controller = position_controller

    def accelerate(self):
        # Only accessible if self.playable = True
        self.speed += CAR_ACCELERATION * SAMPLE_TIME
        self.speed = min(self.speed, CAR_MAX_SPEED)

    def brake(self):
        # Only accessible if self.playable = True
        self.speed -= CAR_ACCELERATION * SAMPLE_TIME
        self.speed = max(self.speed, -CAR_MAX_SPEED)

    def turn_right(self):
        # Only accessible if self.playable = True
        self.steering_wheel_angle += CAR_ANGLE_STEP
        self.steering_wheel_angle = min(self.steering_wheel_angle, MAX_STEERING_WHEEL_ANGLE)

    def turn_left(self):
        # Only accessible if self.playable = True
        self.steering_wheel_angle -= CAR_ANGLE_STEP
        self.steering_wheel_angle = max(self.steering_wheel_angle, -MAX_STEERING_WHEEL_ANGLE)

    def apply_command(self, vr, xr):
        """Method that applies the gets the command of the controllers and
        apply to the vehicle motion"""

        self.steering_wheel_angle = self.position_controller.control(xr, self.position.location.x)
        
        # applies the commanded force to the vehicle
        # the effect of air resistance will be applied only in the self.move() method
        f: float = self.speed_controller.control(vr, self.speed)
        self.speed += f/CAR_MASS * SAMPLE_TIME
        self.speed = min(max(self.speed, -CAR_MAX_SPEED), CAR_MAX_SPEED)

    def move(self):
        theta: float = self.position.rotation
        self.position.location += Vector(sin(theta), cos(theta)) * self.speed * SAMPLE_TIME
        self.position.rotation += self.speed * tan(self.steering_wheel_angle) / CAR_HEIGHT * SAMPLE_TIME
        
        if self.playable:
            # The steering wheel inertia is only applied under user control in order to
            # give a better user experience. The control system do not experience such
            # effect since it is supposed that it has full control of the steering wheel
            self.steering_wheel_angle -= sgn(self.steering_wheel_angle)*INERTIA_PARAMETER_WHEEL
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
        """Returns the bounding box of the vehicle for collision checking"""
        theta = self.position.rotation
        P1 = self.position.location + Vector(sin(theta) * CAR_HEIGHT / 2 - cos(theta) * CAR_WIDTH / 2,
                                             cos(theta) * CAR_HEIGHT / 2 + sin(theta) * CAR_WIDTH / 2)
        P2 = P1 + Vector(cos(theta), -sin(theta)) * CAR_WIDTH
        P3 = P2 + Vector(-sin(theta), -cos(theta)) * CAR_HEIGHT
        P4 = P3 + Vector(-cos(theta), sin(theta)) * CAR_WIDTH
        return Box(P1, P2, P3, P4)
