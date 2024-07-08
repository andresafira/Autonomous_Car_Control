# Self_Driving_Car-CT213_2023

## Name
Self-driving car simulation

## Description
This project aims to use neural networks to train simulated car to avoid obstacles by two different methods: Imitation Learning and Genetic Algorithm.

## Installation

In order to test and use the algorithms and simulation implemented in the project, it is recommended to clone te repository:

`git clone https://gitlab.com/omisso/self_driving_car-ct213_2023.git`

## Usage

### Adjusting simulation parameters

It is possible to adjust the parameters of the simulation, as in the car width, maximum velocity, number of sensors, etc. To do this, simply go to the file "simulation_constants.py" and change the values to better fit the situation you desire.

### Adjusting population parameters

It is also possible to change the genetic algorithm hyperparameters, contained in the file `Genetic_Algorithm\constants.py`, such as
the mutation scale and the crossover reproduction parameter. Numerical parameters, such as the number of mutated sons and number of 
selected classifiers at the end of each generation, can also be changed. The initial parameters given were used to generate the 
classifier represented in `Simulation\Car\best_classifier.txt`.

### Using the simulation

Using the file `Simulation\Car\main_playable.py`, set `learn = False` in order to play around with the simulation.

### Testing the implementation with benchmark functions

In order to test the implementation of the genetic algorithm it is possible to use two scripts: `Simulation\Benchmark\sin_test.py` and
`Simulation\Benchmark\paraboloid_test.py`, which represent a task to train the population to represent the given function. The paraboloid
tes, involves learning how to represent an n-dimensional paraboloid, with given n.

### Training a neural network using Genetic Algorithm

Using the file `Simulation\Car\main_simulation.py`, choose the option 1, and adjust the run options (booleans to convey an action, such as
saving the learned model, creating a population from an existing model, etc). The best classifier can be viewed by setting True the option
`Initialize_pop`. Be aware that if the option `Save_pop` is set to true, at the end of an execution it will override any existing file named
`best_classifier.txt`.

### Training a neural network using Imitation Learning

To train the neural network, first you need to run the code in `Simulation\Car\main_playable.py` to generate a dataset containing the input and expected output. After running it, the simulation will start, and you will be able to control the car with the arrow keys or WASD.
For better results, try to control the car using the same logic for when to do a specific action, such as only changing lanes when a sensor detects another car in the lane you currently are in, rather than doing it as soon as you see it.
If you want a larger dataset, you can change the number of vehicles that appear, go to the file `Simulation\Car\simulation.py` and in line 34 change the number in the definition of `self.dummies`.

To create and train the neural network you created, as well as to test if it is able to properly function, run the code in `Simulation\Car\main_simulation.py` with the integer `option` set as 2 and with the boolean `Train_new_model` set as True. The resulting neural network will be saved in a H5 file with the name chosen in line 40.
You can change how the neural network is defined, as well as the number of epochs and the batch size for training, in the function `get_model`, which is defined in `Simulation\Car\Car.py`.

If you are not satisfied with the results obtained, it is possible to make corrections in the neural network by creating a smaller dataset while running the code in `Simulation\Car\main_simulation.py` with the integer "option" set as 2 and with the boolean `Train_new_model` set as `False`. To do this, use the arrow keys or WASD when a wrong decision is made. After you have closed the simulation, the neural network will be trained again and saved in a H5 file.
For better results, when generating a new dataset for training only press a few times the correct key when the car makes a mistake.

A H5 file containing a functioning neural network trained using Imitation Learning can be loaded in `Simulation\Car\main_simulation.py` is available in the repository as `Imitation.h5`.

## Authors and acknowledgment
André Andrade Gonçalves and Guilherme Saraiva Brasiliense.
