# Mouse

This is where you get to determine what the Vehicle can do. By looking at sensor values and issuing commands, the
Mouse is everything that turns this robot into a maze solving micromouse, a line following robot, a sumo robot or
whatever you desire.

The main application provides the robot ith information about its environment through callbacks or direct manipulation.
The behaviour code reads the current robot state to find out what those interactions are.

The behaviour code should know nothing about the simulation application and so should very closely mimic the code
running on the real device.
