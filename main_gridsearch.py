from simulation import Simulation
from constants import WIND_B, CAR_MAX_SPEED, FREQUENCY, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE, MIDDLE_LEFT, MIDDLE_RIGHT 
from constants import MAX_F_COMMAND, CAR_HEIGHT, CAR_MASS
from Control import PFController, PVController, FullPIDController
import pygame
import sys
import os
import matplotlib.pyplot as plt


def get_speed_constants():
    tau = 0.2
    Kff = WIND_B
    Kx = CAR_MASS/tau - WIND_B
    return Kff, Kx


def get_position_constants(xi, wn, k):
    k0 = CAR_HEIGHT / CAR_MAX_SPEED**2
    kd = k0 * (k + 2) * xi * wn
    kp = k0 * (2 * xi**2 * k + 1) * wn**2
    ki = k0 * k * xi * wn**3
    return kp, ki, kd
    

def get_controllers(xi, wn, k):
    Kff, Kx = get_speed_constants()
    speed_controller = PFController(Kx, Kff, MAX_F_COMMAND)
    kp, ki, kd = get_position_constants(xi, wn, k)
    position_controller = FullPIDController(kp, ki, kd, SAMPLE_TIME, MAX_STEERING_WHEEL_ANGLE)
    
    return speed_controller, position_controller


def simulate(name, xi, wn, k):
    """Performs a grid search for various combinations of values for wn and xi
    in order to determine the better parameters"""
    sim = Simulation(side='left', draw_Bounding_Box=True, draw_reference_line=False, draw_car_line=True)
    sim.car.playable = False
    sim.car.speed = CAR_MAX_SPEED
    run = True

    speed_controller, position_controller = get_controllers(xi, wn, k)
    sim.car.set_controllers(speed_controller, position_controller)
    clock = pygame.time.Clock()
    t: float = 0 # time elapsed

    while run:
        clock.tick_busy_loop(FREQUENCY)
        sim.update()
        xr = MIDDLE_RIGHT
        sim.car.apply_command(CAR_MAX_SPEED, xr)
        t += SAMPLE_TIME
        
        # stop the simulation after 2.5 seconds
        if t > 2.5:
            run = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                sim.reset('left')
    
    # Stores the data in a txt file
    with open(name+".txt", 'w') as file:
        t = 0
        txt = f"xi = {xi}\nwn = {wn}\nk = {k}\nt y\n"
        for x, y in sim.car_points:
            txt += f"{t} {x-MIDDLE_LEFT}\n"
            t += SAMPLE_TIME
        file.write(txt)

def plot_results():
    # Define the range for i and j
    i_range = range(6)
    j_range = range(4)

    # Initialize a dictionary to store data
    data = {}

    # Read the data from files
    for i in i_range:
        for j in j_range:
            filename = f"./grid/grid_{i}_{j}.txt"
            with open(filename, 'r') as file:
                lines = file.readlines()
                xi = float(lines[0].split('=')[1].strip())
                wn = float(lines[1].split('=')[1].strip())
                k = float(lines[2].split('=')[1].strip())
                
                # Initialize the data structure if not already done
                if wn not in data:
                    data[wn] = {}
                if xi not in data[wn]:
                    data[wn][xi] = {'t': [], 'y': []}
                
                # Read the pairs of values starting from the fourth line
                for line in lines[4:]:
                    t, y = map(float, line.split())
                    data[wn][xi]['t'].append(t)
                    data[wn][xi]['y'].append(y)

    # Plot the data
    for wn in data:
        plt.figure()
        for xi in data[wn]:
            plt.plot(data[wn][xi]['t'], data[wn][xi]['y'], label=f'xi = {xi}')
        plt.title(f'Curves for wn = {wn}')
        plt.xlabel('t')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'./grid/graph_wn_{int(wn)}.png')  # Save the figure


if __name__ == "__main__":
    # Here testing the combinations of xi and wn
    i = 0
    for xi in [0.7, 0.8, 0.9, 1, 1.1, 1.2]:
        j = 0
        for wn in [7, 8, 9, 10]:
            simulate(f"./grid/grid_{i}_{j}", xi, wn, 5)
            j += 1
        i += 1

    plot_results()
    sys.exit()
