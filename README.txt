README for swarm code:

V2.0 Greedy Allocation With Global Communication
* When the file 'test.m' is run, a window pops up showing a number of robots and objects
* these robots perform a 360 degree turn before assigning themselves to tasks using the greedy algorithm
* (closes task). No consideration is given to if two robots allocate to the same task

FLIES:

distEU: Calculated the euclidian distacne between any equal sized point sets (A) and (B).

extPoses.m Extracts the pose information from each robot to allow for the environment object to plot them

fixPose.m Normailes any angle (rad) between 0 and 2*pi

flush.m Resets all figures, variables and command line interface for a clean run

getFit.m Returns the fitness of an allocation using the euclidian distance,
			does not yet consider duplicate assignmnet or no assingment 
			(no assingmnet must be of higerfitness than duplicate)
			
greedyA.m Runs a greedy task allocation algorithm to decide each robot's target

height.m Returns the height of a 2d matrix

isIn.m Checks is a single value, A is in an array, B

misc.m Old code

object.m - needs removing.

robot.m Class file for each robot that contians the FSM code used for iterative running

searching.m Code tat manges the robot's inital 360 turn

taskAllo.m rough code for PSO allocation

test.m main code run for simulation. Calls all other functions in its process