import matplotlib.pyplot as plt

from Genetic_Algorithm.Population.Population import Population
from simulation import Simulation
from simulation_constants import FREQUENCY, N_SENSOR, SAMPLE_TIME, CAR_MAX_SPEED, HEIGHT, CAR_HEIGHT
from math import fabs
import pygame
from keras import models


def change_side(side):
    if side == 'left':
        return 'right'
    return 'left'


MAX_SIMULATION_TIME = 20

sim = Simulation(side='left', draw_Bounding_Box=True, draw_Sensors=True)
run = True

option = 1  # for using genetic algorithm
# option = 2  # for using imitation learning

Train_new_model = False  # option to train a new neural network or use an existing one
Initialize_pop = True  # Initialize half of the population with a previous classifier
Save_pop = False  # Save the best classifier found with the method

clock = pygame.time.Clock()
clock.tick(FREQUENCY)
current_time = 0
punish_rate = 0.5  # a rate related to car's tendency to avoid high sensor readings

Pop = Population(N_SENSOR + 1, [2 * N_SENSOR // 3, N_SENSOR // 3], 4)
if option == 1 and Initialize_pop:
    Pop.create_from('best_classifier.txt')

i = 0
score = 0
back_speed = 1
speed = 1

Save_History = False  # Save the fitness history of the classifiers
best_history = []
all_history = []

if option == 2:
    if Train_new_model:
        model = sim.car.get_model()
        model.save('imitation.h5')
    else:
        model = models.load_model('imitation.h5')
        model.save('backup.h5')

while run:
    clock.tick(FREQUENCY)

    # Close the program if the quit button was pressed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            if option == 2:
                print(sim.car.keys_history)
                model.fit(sim.car.sensors_history, sim.car.keys_history, batch_size=16, epochs=500)
                model.save('imitation.h5')
            elif option == 1 and Save_pop:
                Pop.register_best()
            run = False
    back_speed = speed
    sim.update()
    speed = sim.car.speed

    if option == 1:
        if current_time > MAX_SIMULATION_TIME or (not sim.car.alive) or (speed <= 0 and back_speed <= 0):
            back_speed = 1
            speed = 1
            current_time = 0
            score += sim.car.position.location.y - (HEIGHT / 2 - CAR_HEIGHT)

            i += 1
            print('CAR ', i)
            Pop.tell(score)
            if Save_History:
                all_history.append(score)
            if i == Pop.gen_size:
                i = 0
                print('[INFO] Changed generation and best score was ', Pop.best_classifier.fitness)
                if Save_History:
                    best_history.append(Pop.best_classifier.fitness)

            sim.reset(side='left')
            score = 0

        read = sim.car.get_readings()
        input_vector = read + [sim.car.speed / CAR_MAX_SPEED]

        clf_move = Pop.ask(input_vector)

        if clf_move[0] == 1:
            sim.car.accelerate()
        if clf_move[1] == 1:
            sim.car.brake()
        if clf_move[2] == 1:
            sim.car.turn_right()
        if clf_move[3] == 1:
            sim.car.turn_left()

        score -= punish_rate * sum(read)

    if option == 2:
        read = sim.car.get_readings()
        read.append(sim.car.speed / CAR_MAX_SPEED)
        input_vector = [read]

        clf_move = model.predict(input_vector)[0]

        if clf_move.argmax() == 0:
            sim.car.accelerate()
        if clf_move.argmax() == 1:
            sim.car.turn_left()
        if clf_move.argmax() == 2:
            sim.car.turn_right()
        if clf_move.argmax() == 3:
            sim.car.brake()

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            sim.car.keys_history.append([1, 0, 0, 0])
            sim.car.sensors_history.append(read)
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            sim.car.keys_history.append([0, 1, 0, 0])
            sim.car.sensors_history.append(read)
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            sim.car.keys_history.append([0, 0, 1, 0])
            sim.car.sensors_history.append(read)
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            sim.car.keys_history.append([0, 0, 0, 1])
            sim.car.sensors_history.append(read)

    current_time += SAMPLE_TIME


if option == 1 and Save_History:
    plt.plot(best_history)
    plt.show()
    plt.plot(all_history)
    plt.show()
