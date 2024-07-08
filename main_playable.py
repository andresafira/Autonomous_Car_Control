from simulation import Simulation
from simulation_constants import FREQUENCY, CAR_MAX_SPEED
import pygame


def main():
    sim = Simulation(side='left', draw_Bounding_Box=True)
    run = True

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        sim.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            sim.car.accelerate()
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            sim.car.turn_left()
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            sim.car.turn_right()
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            sim.car.brake()

if __name__ == "__main__":
    main()
