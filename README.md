# Robot Motion Planning

This project implements a virtual robot that is capable of exploring and mapping a given, unfamiliar Micromouse maze, identifying an optimal path from the maze's corner to the maze's center, and traveling along the optimal path to reach it's goal using the least amount of time possible. 

### Install

Python 2.7.X
Numpy

### Files

robot.py - This script establishes the robot class.
maze.py - This script contains functions for constructing the maze and for checking for walls upon robot movement or sensing.
tester.py - This script will be run to test the robot’s ability to navigate mazes.
showmaze.py - This script can be used to create a visual demonstration of what a maze looks like.
test_maze_##.txt - These files provide the sample mazes upon which to test the robot.

### Example
To run the tester, you can do so from the command line with a command like the following: 
`python tester.py test_maze_01.txt. `

The maze visualization follows a similar syntax, e.g. 
`python showmaze.py test_maze_01.txt.`

The script uses the turtle module to visualize the maze; you can click on the window with the visualization after drawing is complete to close the window.
