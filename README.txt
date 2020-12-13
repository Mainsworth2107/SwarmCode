All references are given in main.m

README for DABC code

FLIES:

beeFit: Used the fitness function outlined by Jevtic et al to calculate solution fitness

beeShift: Function that applies one point mutation to current solutions to produce new potential solutions

distEU: Calculates the Euclidian distance between any equal sized point sets (A) and (B).

extPoses.m Extracts the pose information from each robot to allow for the environment object to plot them.

flush.m Resets all figures, variables and command line interface for a clean run.

height.m Returns the height of a 2d matrix.

initBots.m Initialises a random robot placement to mimic a single allocation situation within the simulation.
 
main.m main code run for simulation. Calls all other functions in its process.

robot.m Class file for each robot that contains the FSM code used for iterative running.

shiftSol.m Handels the overall solution shifting behaviour in both the employed and onlooker bees phases

updateSol.m Applies the greedy selection method as outlined by Karaboga to choose between the current and
	potential solution
