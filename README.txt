README for RDBA code

Full refernces are given in main.m

FLIES:

distEU Calculates the euclidian distance between any equal sized point sets (A) and (B).

beeNew.m Function that applies swapping and mutation to  randomly selected current solutions to produce new potential solutions

extPoses.m Extracts the pose information from each robot to allow for the environment object to plot them.

flush.m Resets all figures, variables and command line interface for a clean run.

height.m Returns the height of a 2d matrix.

initBots.m Initialises a random robot placement to mimic a single allocation situation within the simulation.
 
main.m main code run for simulation. Calls all other functions in its process.

newfit.m: Uses the method described in Andina et al to produce an allocation for each robot in the swarm.

robot.m Class file for each robot that contains the FSM code used for iterative running.

shiftSol.m Handles the overall solution shifting behaviour in both the employed and onlooker bees phases

updateSol.m Applies the greedy selection method as outlined by Karaboga to choose between the current and
	potential solution

