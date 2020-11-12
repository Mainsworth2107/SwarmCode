README for DBA code

DBA Algorithm (Jevtić et al): Implementation and verification of DBA Algorithm

FILES:

DBAFit.m: Uses the method described in Jevtić et al to produce an allocation for each robot in the swarm.

distEU: Calculates the Euclidian distance between any equal sized point sets (A) and (B).

extPoses.m Extracts the pose information from each robot to allow for the environment object to plot them.

flush.m Resets all figures, variables and command line interface for a clean run.

height.m Returns the height of a 2d matrix.

initBots.m Initialises a random robot placement to mimic a single allocation situation within the simulation.
 
main.m main code run for simulation. Calls all other functions in its process.

robot.m Class file for each robot that contains the FSM code used for iterative running.

