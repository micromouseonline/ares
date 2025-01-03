# Vehicle

In here should be those things that concern the physical robot. The vehicle if you like.

All the hardware dependencies will live here as well as the low-level control mechanisms.

The robot state is a structure holding a view of all the accessible components of the robot. Everything from the current
pose to the state of the buttons and the battery voltage.

The robot state can be accessed in one big chunk only. Another option would be for it to be published to a queue for
processing elsewhere.

One or more callbacks are used to update the environment sensors.

UI interface allow the user to set the state of buttons, encoders or LEDs from the simulation, to represent interaction
with the user.

The robot has very few controls. Essentially, you need only issue a forward and angular velocity. For ideal simulation
purposes, assume that those velocities are instantaneously met with perfect tracking.

A more complex physics model might include inertia, tyre slip, motor capabilities, battery properties and so on. even
so, control is still by means of sending a pair of desired velocities.
