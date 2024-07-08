from simulation_constants import WIDTH, HEIGHT, CAR_HEIGHT, CAR_WIDTH, BACKGROUND_SPRITE, CAR_SPRITE
from simulation_constants import CAR_START_LEFT, CAR_START_RIGHT
from simulation_constants import SAMPLE_TIME, eps, SIDEWALK_WIDTH, MIDDLE_RIGHT, MIDDLE_LEFT
from simulation_constants import BLACK, WHITE, RED
from Utils.General import clip
from Utils.Geometry.Box import *
from Utils.Geometry.Position import Position
import pygame
from pygame.transform import scale, rotate
from pygame.image import load
from pygame.locals import *
from Car import Car
from math import pi, sin, cos, fabs


def make_default_position(location):
    return Position(Vector(location[0], location[1]), 0)


def dummy_simple_generator(num_dummy, step=500, side='right'):
    if side == 'left':
        first = MIDDLE_RIGHT
        second = MIDDLE_LEFT
    elif side == 'right':
        first = MIDDLE_LEFT
        second = MIDDLE_RIGHT
    else:
        raise Exception('Invalid side choice! Choose left or right side')
    dummies = []
    for i in range(num_dummy):
        if i % 2 == 0:
            pose = Position(Vector(first, HEIGHT/2 - CAR_HEIGHT + i * step), 0)
        else:
            pose = Position(Vector(second, HEIGHT/2 - CAR_HEIGHT + i * step), 0)
        dummies.append(Car(initial_position=pose, DUMMY=True, initial_speed=10))
    return dummies


class Simulation:
    def __init__(self, side: str = 'left', draw_Bounding_Box: bool = False):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Car simulation")
        self.background_sprite = load(BACKGROUND_SPRITE)
        self.car_sprite = scale(load(CAR_SPRITE), (CAR_WIDTH*1.1, CAR_HEIGHT))
        if side == 'left':
            self.car = Car(make_default_position(CAR_START_LEFT))
        elif side == 'right':
            self.car = Car(make_default_position(CAR_START_RIGHT))
        else:
            raise Exception('Invalid side choice! Choose left or right side')
        self.dummies = dummy_simple_generator(30, side=side)
        self.objects = []
        self.draw_BB = draw_Bounding_Box
        self.update_check = 0

    def reset(self, side: str = 'left'):
        self.car.speed = 0
        self.car.alive = True
        self.dummies.clear()
        self.update_check = 0
        if side == 'left':
            self.car.position = make_default_position(CAR_START_LEFT)
            self.dummies = dummy_simple_generator(20, side='left')
        elif side == 'right':
            self.car.position = make_default_position(CAR_START_RIGHT)
            self.dummies = dummy_simple_generator(20, side='right')
        else:
            raise Exception('Invalid side choice! Choose between left or right side')
        self.update_objects()

    def update_objects(self):
        self.objects.clear()
        for dummy in self.dummies:
            dummy.update()
        # ROAD Segment:
        left_side = Segment(Vector(SIDEWALK_WIDTH, self.car.position.location.y - CAR_HEIGHT),
                            Vector(SIDEWALK_WIDTH, self.car.position.location.y + CAR_HEIGHT))
        right_side = Segment(Vector(WIDTH - SIDEWALK_WIDTH, self.car.position.location.y - CAR_HEIGHT),
                             Vector(WIDTH - SIDEWALK_WIDTH, self.car.position.location.y + CAR_HEIGHT))
        self.objects.append(left_side)
        self.objects.append(right_side)
        for dummy in self.dummies:
            self.objects.append(dummy.bounding_box)

    def draw_bounding_box(self, car: Car):
        box = car.unravel_box()
        P1 = box.vertices[0].x, HEIGHT/2 - box.vertices[0].y + self.car.position.location.y + CAR_HEIGHT / 2
        P2 = box.vertices[1].x, HEIGHT/2 - box.vertices[1].y + self.car.position.location.y + CAR_HEIGHT / 2
        P3 = box.vertices[2].x, HEIGHT/2 - box.vertices[2].y + self.car.position.location.y + CAR_HEIGHT / 2
        P4 = box.vertices[3].x, HEIGHT/2 - box.vertices[3].y + self.car.position.location.y + CAR_HEIGHT / 2

        pygame.draw.line(self.screen, WHITE, P1, P2)
        pygame.draw.line(self.screen, WHITE, P2, P3)
        pygame.draw.line(self.screen, WHITE, P3, P4)
        pygame.draw.line(self.screen, WHITE, P4, P1)

    def draw_car(self, car: Car):
        sprite_copy = rotate(self.car_sprite, car.position.rotation * 180 / pi)
        if not self.car.alive:
            sprite_copy.set_alpha(50)
        self.screen.blit(sprite_copy, (car.position.location.x - sprite_copy.get_width() / 2,
                                       HEIGHT / 2 - car.position.location.y + self.car.position.location.y))
        if self.draw_BB:
            self.draw_bounding_box(car)

    def draw_scenario(self):
        self.screen.blit(self.background_sprite, (0, clip(self.car.position.location.y, HEIGHT)))
        self.screen.blit(self.background_sprite, (0, clip(self.car.position.location.y - HEIGHT, -HEIGHT)))
        for dummy in self.dummies:
            self.draw_car(dummy)
        self.draw_car(self.car)

    def update(self):
        self.screen.fill(BLACK)
        if self.update_check != 0:
            self.objects.clear()
            self.update_check = (self.update_check + 1) % 3
        self.car.update(self.objects)
        self.draw_scenario()
        self.update_objects()
        pygame.display.flip()
