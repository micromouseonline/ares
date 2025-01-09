# ARES architecture

The simulator has a main Application class. The Application contains a RobotManager that looks after the actual MR32
mouse behaviour code. The MR32 code is in the behaviour directory. All this is in keeping with the Robot = Vehicle +
Behaviour architecture.

The Application creates instances of the Vehicle and the Behaviour and gives them to the Robot Manager.

The Vehicle will have buttons and LED displays on it as well as the sensors.

The Application needs to be able to simulate the pushing of the buttons on the Vehicle and light its indicator LEDs.
Maybe it will even simulate beeping the on-board speaker. This should be among the few ways the simulator can talk to
the Vehicle. There is also the need for a callback form the Vehicle to get sensor data. Since that must happen every
millisecond anyway, the sensor request could send the LED state and the receiver sensor data package could have in it
the new state of the buttons and LEDS. That way there is only one interface between the Vehicle and the Application and
I believe that would be intrinsically thread safe.

The mouse Behaviour has a reference to the Vehicle.

The mouse behaviour will be able to read the buttons from the Vehicle, light its LEDS and, possibly, tell it it to beep.
This will be by direct calls to Vehicle methods.

When the user interacts with the simulator, ARES, it should really be via the virtualised button presses but many actual
robots have on-board menu systems for function selection. there should probably be a way to give the Vehicle a function
code to set it up as if you had selected an option via the wheels or on-board menu. It can then be activated by a button
press.

These indirections mimic functionality on typical real targets and will make it easier for the mouse Behaviour to run
unchanged.

That last part of the puzzle then will be the generation and collection of log messages. If the mouse Behaviour has a
suitable logging class, it could use conditional compilation to send its output to on-board devices/storage or to the
RobotManager which can make it available to the main Application.

The Robot Manager is what starts the mouse behaviour running in a thread. After that there should be no direct
communication between threads except that logging output and the simulated sensors.
