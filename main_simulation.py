from simulation import Simulation
from constants import WIND_B, CAR_MAX_SPEED, FREQUENCY, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE, MIDDLE_LEFT, MIDDLE_RIGHT 
from constants import MAX_F_COMMAND, CAR_HEIGHT, CAR_MASS
from Control import PFController, FullPIDController
import pygame
import sys


def get_speed_constants() -> tuple[float, float]:
    """Funtion the returns the constants for the speed control system"""
    tau = 0.2
    Kff = WIND_B
    Kx = CAR_MASS/tau - WIND_B
    return Kff, Kx


def get_position_constants() -> tuple[float, float, float]:
    """Funtion the returns the constants for the position control system"""
    xi = 1
    wn = 10
    k0 = CAR_HEIGHT / CAR_MAX_SPEED**2
    k = 5
    kd = k0 * (k + 2) * xi * wn
    kp = k0 * (2 * xi**2 * k + 1) * wn**2
    ki = k0 * k * xi * wn**3
    return kp, ki, kd


def get_controllers() -> tuple[PFController, FullPIDController]:
    """Function that returns the car controllers"""
    Kff, Kx = get_speed_constants()
    speed_controller = PFController(Kx, Kff, MAX_F_COMMAND)

    kp, ki, kd = get_position_constants()
    position_controller = FullPIDController(kp, ki, kd, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE)
    
    return speed_controller, position_controller


def main():
    """Performs the simulation of the system dynamics for the chosen controllers.
    The main car starts at rest on the wrong lane, so that the effects of the speed
    change can be observed, for the position controller. After that dummy cars will
    appear in order to test the reaction system."""

    sim: Simulation = Simulation(side='left', draw_Bounding_Box=True, draw_reference_line=True, draw_car_line=True)
    # Ensures that only the controllers will affect the car movement
    sim.car.playable = False
    run: bool = True

    speed_controller, position_controller = get_controllers()
    sim.car.set_controllers(speed_controller, position_controller)
    clock = pygame.time.Clock()
    t: float = 0
    
    while run:
        clock.tick_busy_loop(FREQUENCY)
        sim.update()
        vr, yr = sim.get_reference_parameters()
        sim.car.apply_command(vr, yr)
        
        # time increment
        t += SAMPLE_TIME
        if t > 3:
            t = 0
            # add car in the right lane
            sim.generate_dummy(MIDDLE_RIGHT, CAR_MAX_SPEED/2)


        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                sim.reset('left')


if __name__ == "__main__":
    main()
    sys.exit()
