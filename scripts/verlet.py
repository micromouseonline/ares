import matplotlib.pyplot as plt
import numpy as np

'''
This script models a harmonic oscillator using two
different integration schemes. One is standard Euler
integration, the other is basic Verlet integration.

These two techniques will produce very similar
results for simple cases with constant forces such as
linear motion. However, when the forces are varying
Euler integration can generate errors that may grow
with time. By using Verlet integration, the errors
will be smaller and more stable.

In this example, the Euler method causes the oscillations 
to grow exponentially whereas the Verlet method is stable.

Note that reducing the delta time (dt) can make the 
Euler method more stable but with a corresponding
increase in the processing time needed.
'''


def euler_integration(k, mass, dt, total_time):
    num_steps = int(total_time / dt)
    t = np.linspace(0, total_time, num_steps)

    # Initialize arrays for acceleration, velocity, and position
    a = np.zeros(num_steps)
    v = np.zeros(num_steps)
    x = np.zeros(num_steps)

    # Initial conditions
    x[0] = 1.0  # Initial position
    v[0] = 0.0  # Initial velocity

    for i in range(1, num_steps):
        a[i - 1] = -k * x[i - 1] / mass

        v[i] = v[i - 1] + a[i - 1] * dt
        x[i] = x[i - 1] + v[i - 1] * dt

    return t, x, v


def verlet_integration(k, mass, dt, total_time):
    num_steps = int(total_time / dt)
    t = np.linspace(0, total_time, num_steps)

    # Initialize arrays for acceleration, velocity, and position
    # basic Verlet does not calculate speed
    a = np.zeros(num_steps)
    v = np.zeros(num_steps)
    x = np.zeros(num_steps)

    # Initial conditions
    x[0] = 1.0  # Initial position
    x[1] = x[0] + dt * 0.0  # Assuming initial velocity is 0, we can set x[1] appropriately

    for i in range(1, num_steps - 1):
        a[i] = -k * x[i] / mass
        x[i + 1] = 2 * x[i] - x[i - 1] + a[i] * dt ** 2

    # Calculate velocity using central difference
    for i in range(1, num_steps - 1):
        v[i] = (x[i + 1] - x[i - 1]) / (2 * dt)

    v[0] = (x[1] - x[0]) / dt  # Forward difference for the first element
    v[-1] = (x[-1] - x[-2]) / dt  # Backward difference for the last element
    return t, x, v


# Simulation parameters
mass = 1.0
k = 10.0  # Spring constant
dt = 0.01
total_time = 20.0

# Run simulations
t_euler, x_euler, v_euler = euler_integration(k, mass, dt, total_time)
t_verlet, x_verlet, v_verlet = verlet_integration(k, mass, dt, total_time)

# Plot results
plt.plot(t_euler, x_euler, label='Euler Integration')
plt.plot(t_verlet, x_verlet, label='Verlet Integration')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Position vs. Time for a Harmonic Oscillator')
plt.legend()
plt.grid()
plt.show()
