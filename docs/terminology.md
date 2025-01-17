# Terms used in the behaviour code

## Path, Trajectory, Profile

In the context of mobile robotics and autonomous systems, the terms path, trajectory and profile have distinct but
related roles. While they are sometimes used interchangeably, each represents a different aspect of the planning and
execution of motion.

A Path defines the geometric route a robot should follow to reach its destination, focusing on the spatial arrangement
of waypoints. A Trajectory, on the other hand, incorporates the element of time, specifying how the robot moves along
the path while considering kinematic and dynamic constraints. Finally, a Profile governs the velocity and acceleration
over time, ensuring smooth transitions and adherence to physical limitations.

## Path:

- **Definition**: A path is a geometric representation of the route that a robot or vehicle should follow to reach its
  destination. It is generally a sequence of connected points in space, usually defined by coordinates (x, y) in a 2D
  space. These waypoints must be visited in a particular order. If the waypoints are very close to each other, the path
  can be followed using techniques like pure-pursuit. Waypoints that are widely spaced will need to be connected with,
  ideally, smooth trajectories

- **Planning**: Path planning focuses on finding an optimal route from the starting point to the goal, considering
  obstacles and environmental constraints. For this you might use algorithms like Dijkstra, A* or some other domain
  specific method.

- **No Time Dependency**: Paths do not consider time, speed, or dynamics. They are purely spatial representations.

## Trajectory:

- **Definition**: A trajectory is a time-parameterized representation of the robot's motion along a path between two
  waypoints. It specifies the position, velocity, and sometimes acceleration of the robot at each point in time along
  that path. It includes the coordinates (x, y), orientation (theta), and time-dependent elements like velocity and
  acceleration.

- **Planning**: Trajectory planning involves generating a feasible motion profile that the robot can follow, given its
  kinematic and dynamic constraints. This includes considerations of speed, acceleration, and time.

- **Time Dependency**: Trajectories incorporate the dimension of time, detailing how the robot moves through space over
  time.

## Profile:

- **Definition**: A profile generally describes how velocities and accelerations change over time to determine the
  motion of a robot or vehicle and ensure that it follows the desired trajectory. Velocity profiles ensure smooth
  acceleration, turning and braking. Acceleration components serve to ensure smooth transitions and to avoid jerky
  movements.

- **Planning**: Given the constraints of the trajectory and the dynamics of the vehicle or robot, velocity and
  acceleration profiles can be precomputed in their entirety or generated as a function of elapsed time or distance.
  these might be simple triangular or trapezoidal profiles for straight line motion or in place turns for example.
  These calculations can also take into account other constraints such as available torque or maximum permitted speeds.

- **Control**: During execution of a profile, external control inputs may modify the computed profile to handle
  requirements such as steering correction or obstacle avoidance.

## Key Points:

- Paths describe the route between two endpoints
- Paths are for high-level planning and, while not time dependent, they can be evaluated for their duration or cost in
  order to select optimal routes.
- Trajectories connect the waypoints along a path. They have a time dependence and are designed for smooth transitions.
- Profiles generate the velocities and accelerations that implement the trajectories

## Implementation

- **Path Generation**: In a micromouse, the path generator will examine the starting and ending locations and create a
  set
  of simple movements through cells which, ideally, will result in the fastest possible route. Typically that might
  consist of simple primitives that can be expressed in a list like a string. For example, the string "FFFFRRFFLFF"
  might mean four forward moves, a right turn, a right turn, two forward moves, a left turn, two forward moves and an
  implied halt. Stepping through this list, the robot can move from start to goal by means of a series of discrete
  moves.

- **Path Optimisation**: By combining primitives from the path, it is possible to generate a list of individual motions
  which are trajectories. The four forward moves at the start of the previous path becomes a single straight line
  motion; the two successive right turns merge into a single smooth 180 degree turn and so on. This process ensures
  transitions between trajectories are smooth, reducing jerk and enhancing the robot's stability. Special cases, like
  predefined maneuvers, can also be recognized and incorporated for efficiency.A key part of that conversion is to
  ensure that transitions between two trajectories are smooth and continuous. In the case of a micromouse, this is
  generally made easier by the observation that straight line motions always happen at multiples of 45 degrees so that
  we can design trajectories with appropriate start and exit orientations to match

- **Trajectory Execution**: Under the control of trajectory manager, the individual trajectories are
  executed from a list or queue. For each, a suitable velocity profile (forward and angular) is used which is updated
  regularly and the resulting velocities are sent to the vehicle. Acceleration profiling can also be incorporated for
  the reduction of jerk. The calculations needed to implement the profiles are most conveniently contained within the
  trajectory object so that everything is updated consistently. For the purposes of this document it is enough to
  assume that the vehicle will be able to accurately provide the commanded velocities. Sensor inputs and control
  algorithms will provide correction terms that are fed back into the profiler before velocity commands are sent to the
  vehicle.
