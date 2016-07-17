# h's homemade Motion Control: firmware for Teensy-driven DC Servo motor axes
Introduction

This code is designed to be uploaded to a Teensy LC microcontroller, which can then be hooked up via a suitable motor driver
to a DC servo motor. A quad encoder will also need to be wired to the Teensy. The microcontroller can then be connected to a Mac via USB, and the Motion Control
Server app run, which will allow full (and kinematically safe) manual control of the motor via on-screen sliders, and the playback of 
sequences created in Blender.

This firmware performs PID calculations, using input from a quad encoder in order to drive the motor to the desired target position.

This code does nothing on its own, and needs the Mac app in order to do anything useful. Note that the motor (and its associated mechanics)
will need to be tuned appropriately. There is code included in this firmware to allow the channel to be put into a PID tuning mode
which feeds back a variety of parameters and measurements to the host computer; this needs my (similarly badly coded) PID tuning software
which I'll stuff up here on GitHub shortly. Don't hold your breath, it too is held together with chewing gum and prayers.

To see it working, watch this: https://www.youtube.com/watch?v=rWuKmWKicro
For the server code, see this project: https://github.com/howiemnet/MotionControl

Warnings

This code contains more bugs than working lines of code. I will not be offering support or help in understanding
what the hell is going on beyond what I can get written up for my blog. It's just too big and complex a project: it relies 
on not just this app working correctly, but the right hardware wiring up, and the appropriate firmware running on
the various microcontrollers involved.

That said, it's my intention to try and tidy this up and document it well enough that the project will be reproducable, in the 
hope that others may be able to hack together motion control systems without having to re-invent these wheels.

A lot more documentation will come: this is just the first commit to get it all up there - feel free to poke around and laugh
at my appalling coding style. Just keep it to yourself, y'all

:)

h 17/7/2016
