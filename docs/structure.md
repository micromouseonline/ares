# Summary of the Application Structure

1. Vehicle:  is a model of the physical vehicle.
    - At its simplest, it is given trajectory information and it turns that into (virtual) motion.
    - Everything that happens is tied to a core operation - Systick. In there all the motion calculations happen.
    - Systick also generates sensor requests which are service by the Application. Probably through a callback
    - A more advanced implementation might involve full physics simulation
    - Sensor position and orientation is stored by the robot
    - Sensor data is held by the Vehicle. Interpretation of that data is not its concern.
    - Systick maintains internal time - one tick per iteration
    - Systick adds to a list of state information on every iteration. That is effectively a black box recorder running
      at the systick frequency
1. Behaviour: Combined with Vehicle, that is what makes this a micromouse simulation
    - Behaviour is asynchronous in the sense that the behaviour code must wait for specific states or flags in the
      Vehicle
    - Vehicle never tells Behaviour what to do.
    - Behaviour tells the Vehicle what to do and constantly monitors its achievement.
    - Behaviour does not care about the world time, If it cares at all it is about tick count from the robot.
    - This dependence means that robot can run systick at arbitrary speed but behaviour can timestamp its actions using
      the robot tick count.
    - Behaviour also generates a log of actions. For simplicity, this can be separate to the Vehicle black box but the
      common time stamp lets you sync them up later.
1. World: is where the information about the physical environment is managed
    - Knows about the location and dimensions of the walls an posts
    - A Line follower would have the track layout
    - Can load, save and edit the environment.
1. Application: Is really the simulator framework
    - Coordinates interaction between the Vehicle, Behaviour and the World.
    - Handles sensor request callbacks
    - Handles User Interface.
    - Can update parameters in the Vehicle
    - Can set Behavioural goals or parameters
    - Can read shared state from Vehicle and Behaviour
    - Provides visualisation by displaying state held in the black box record and the behaviour log.

That last item is, I think, pretty central in terms of one of the logging/black box requirements

If the Behaviour and the Vehicle run as fast as the computer permits, it can spew out the black box and behaviour logs
very quickly. There is no need to try and make the overall operation run at 1:1 real time.

Suppose the systick frequency is 1kHz. One mode in the Application would be to just run through the data, displaying a
new frame every 16/17 items. That would look like a real-time run of the robot.


---

(written by ChatGPT)

The application simulates a robot navigating through a maze, with sensor data being used for path planning and
navigation. It is designed to be multi-threaded with a separation of concerns between different components such as the
robot, sensors, maze, and display. Here's a breakdown of the main classes, their roles, and some advantages and
disadvantages of the structure.

## Main Classes

1. Application

- Role: The central controller for the simulation. It manages the overall simulation process, including:
    - Handling user input (e.g., keyboard events).
    - Updating and rendering the simulation.
    - Managing the maze and calculating sensor data based on the robot's position and orientation.
    - It owns the Vehicle instance and uses it to request sensor readings.
- Key Methods:
    - Run(): Starts the application, continuously updating and rendering the simulation.
    - SetRobot(): Assigns the Vehicle instance.
    - CallbackCalculateSensorData(): Calculates the latest sensor readings based on the robot's position and orientation
      in
      the maze.
    - GetSensorValues(): Returns the most recent sensor data to the robot.
- Attributes:
    - m_vehicle: A pointer to the Vehicle instance.
    - m_maze: The maze in which the robot navigates.
    - sensor_data: Stores the calculated sensor readings.
- Advantages:
    - Centralized control of the simulation.
    - Manages interaction between various components (e.g., maze, sensors).
    - Provides an easy way to update and render the simulation loop.
- Disadvantages:
    - Can become complex as more features are added.
    - Tight coupling between Application and Vehicle (though mitigated with forward declarations and pointers).

2. Vehicle

- Role: Represents the robot in the simulation. It is responsible for moving around the maze and requesting sensor
  updates
  from the `Application`.
- Key Methods:
    - start(): Starts the robot and the sensor update thread.
    - Stop(): Stops the robot and the sensor update thread.
    - UpdateSensors(): Requests the latest sensor data from the Application.
- Attributes:
    - m_position: Current position of the robot in the simulation.
    - m_orientation: Current orientation of the robot (in degrees).
    - sensor_data: Stores the sensor readings received from the Application.
    - m_sensorUpdateThread: A separate thread that periodically requests sensor data from the Application.
    - m_application: A pointer to the Application to fetch sensor data.
- Advantages:
    - Decouples the robot logic from the main application loop.
    - The sensor update thread operates independently of the main simulation loop, allowing for faster updates.
- Disadvantages:
    - Needs to handle thread synchronization when accessing shared data between threads.
    - Requires explicit management of thread lifecycle (e.g., start/stop).

3. SensorValues

- Role: A struct used to store the sensor readings (such as distance data) from the environment.
- Attributes:
    - readings: A vector that stores the sensor readings.
- Advantages:
    - Simplifies the data structure for sensor readings, making it easy to pass around.
- Disadvantages:
    - Limited flexibility; if additional sensor data types or more complex structures are needed, this might require
      changes.

4. MazeManager

- Role: Holds the map of the maze and provides functionality for calculating sensor readings based on the robot's
  position and orientation.
- Key Methods:
    - GetSensorReading(): Calculates the sensor reading (e.g., distance) based on the robot’s current position,
      orientation,
      and the maze layout.
- Advantages:
    - Encapsulates maze-related logic, making it easier to update the maze and how the robot interacts with it.
- Disadvantages:
    - The Application class needs access to the maze to calculate sensor readings, which adds some coupling.

5. RobotControl

- Role: Controls the movement of the robot (e.g., linear velocity and angular velocity). This would interface with the
  Vehicle to update the robot's motion based on planned trajectories.
- Advantages:
    - Decouples robot control from the robot's actual movement and sensors, allowing for easier updates to the control
      logic
      without affecting the rest of the system.
- Disadvantages:
    - Still needs integration with other components like the sensor system for real-time updates.

6.RobotDisplay

- Role: Responsible for rendering the robot's state, including its position and orientation, onto the simulation window.
- Advantages:
    - Decouples the drawing/rendering logic from the core robot and application functionality.
- Disadvantages:
    - Could become more complex if additional visualization features (e.g., path planning, sensor readings) are needed.

---

## Advantages of the Structure

1.Separation of Concerns:

The structure separates the different responsibilities (e.g., robot logic, sensor updates, maze management, rendering).
Each class has a clear and distinct role, which improves modularity and maintainability.

2. Multi-threading:

- The sensor updates are handled in a separate thread, which allows for fast sensor data acquisition without blocking
  the
  main simulation loop. This improves performance, especially for simulations that require frequent sensor updates (
  e.g.,
  100 Hz).

3. Encapsulation:

- Each class encapsulates its own behavior and state. For example, the robot handles its movement, the maze handles map
  generation, and the application handles the overall flow. This makes it easier to extend and test individual
  components.

4. Flexibility:

- The design allows for easy adjustments, such as adding new sensors or control strategies without affecting the other
  components.

---

## Disadvantages of the Structure

1. Thread Synchronization:

- The use of multiple threads (e.g., for sensor updates) introduces complexity in managing shared resources and
  synchronization. If not handled properly, this could lead to race conditions or data inconsistencies.

2. Tight Coupling Between Application and Vehicle:

- Although the circular dependency has been resolved with forward declarations, the Application still has a direct
  dependency on the Vehicle for sensor updates. This can make the system harder to maintain as it grows.

3. Complexity:

- As the application adds more features (e.g., advanced path planning, more sensors, etc.), the interactions between
  classes could become more complex. This might make the code harder to scale and debug.

4. Potential for Overhead:

- The separate threads for sensor updates could introduce overhead, particularly if there are many sensors or the thread
  management is inefficient.

---

## Conclusion

The structure of the application is well-designed for modularity and scalability, with clear separation of
responsibilities across different classes. However, challenges related to multi-threading and potential coupling between
the Application and Vehicle classes may arise as the system grows. Thread synchronization and the management of shared
data are key areas to watch as the application evolves.
