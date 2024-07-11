from simulation import Simulation
from constants import WIND_B, CAR_MAX_SPEED, FREQUENCY, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE, MIDDLE_LEFT, MIDDLE_RIGHT 
from constants import MAX_F_COMMAND, CAR_HEIGHT
from Control import PFController, PVController, FullPIDController
import pygame
import sys


def get_speed_constants():
    Kff = WIND_B
    Kx = 4000
    return Kff, Kx

def get_position_constants(isPID=True):
    xi = 0.9
    wn = 6.4462
    if isPID:
        k0 = CAR_HEIGHT / CAR_MAX_SPEED**2
        k = 5
        kd = k0 * (k + 2) * xi * wn;
        kp = k0 * (2 * xi**2 * k + 1) * wn**2;
        ki = k0 * k * xi * wn**3;
        return kp, ki, kd
    
    kp = wn/(2*xi*CAR_MAX_SPEED)
    kv = 2*xi*wn*CAR_HEIGHT/CAR_MAX_SPEED
    return kp, kv


def get_controllers():
    Kff, Kx = get_speed_constants()
    speed_controller = PFController(Kx, Kff, MAX_F_COMMAND)

    PIDControl = False

    if PIDControl:
        kp, ki, kd = get_position_constants(PIDControl)
        position_controller = FullPIDController(kp, ki, kd, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE)
    else: 
        kp, kv = get_position_constants(PIDControl)
        position_controller = PVController(kp, kv, MAX_STEERING_WHEEL_ANGLE)    
    
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
