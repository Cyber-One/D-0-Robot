# D-0-Robot

The D-0 robot was a robot character from the 9th movie of the Star Wars franchise, now owned by Disney.

This robot was built and programmed before the official release of the movie.

You can see a video of the build here:
https://youtu.be/lvXu4mKY4aM

and a video of it out and about at the local fair:
https://youtu.be/AY0BQnsvyXI

The robot is powered by a pair of LiIon batteries, a SMPS to get 5V for the Arduino and to power the small PM3 Player.
The wheely is actually 3 parts, a thin wheel on each side that make contact with the ground, and a larger wheel suspensded just above the wheel.
This give differential steering while appearing to be a single wheel.
Balance is maintained by lead shot in the bottom of the frame inside the robot.
The wheels are driven by a pair of contiuos rotation servos controlled by the Arduino Nano
Also hiding inside the robot is a FlySky IA6 6 channel receiver with the iBus serial servo port connected to the Arduino Nano.
This allows the full 10 channels from the transmitter to be used.
All the servo control mixing was done in the Arduino Nano.
