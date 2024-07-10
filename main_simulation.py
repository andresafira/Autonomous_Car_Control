from simulation import Simulation
from constants import WIND_B, CAR_MAX_SPEED, FREQUENCY, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE, MIDDLE_LEFT, MIDDLE_RIGHT 
from constants import MAX_F_COMMAND
from Control import PFController, PVController
import pygame
import sys

def get_controllers():
    # Setting speed controller
    Kff = WIND_B
    Kx = 2000
    speed_controller = PFController(Kx, Kff, MAX_F_COMMAND)

    xi = 0.9
    wn = 10
    position_controller = PVController(xi, wn, MAX_STEERING_WHEEL_ANGLE)
    return speed_controller, position_controller


def main():
    sim = Simulation(side='left', draw_Bounding_Box=True, n_dummies=0)
    run = True

    speed_controller, position_controller = get_controllers()
    sim.car.set_controllers(speed_controller, position_controller)
    clock = pygame.time.Clock()
    i = 0
    flip = True
    while run:
        i += 1
        a = clock.tick_busy_loop(FREQUENCY)
        sim.update()
        if i > 5*180:
            i = 0
            flip = not flip
        sim.car.apply_command(CAR_MAX_SPEED, MIDDLE_RIGHT if flip else MIDDLE_LEFT)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                sim.reset()


if __name__ == "__main__":
    main()
    sys.exit()
