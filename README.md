Chair I2C Interface
===================
Copyright 2020 SuperHouse Automation Pty Ltd <www.superhouse.tv>  
Designed by Jonathan Oxer

Connects to a Permobil electric wheelchair controller, and emulates the
signals from a wheelchair joystick so that the chair can be controlled
electronically. Provides an I2C interface for connection to a host
microcontroller.

Also exposes the "5th Button" and 12V 100mA chair power connections.

![PCB](Images/CHAIRI2C-v1_0-oblique.jpg)

Features:

 * 1 x chair controller output.
 * I2C interface.
 * Derives 12V power from the host wheelchair controller.
 * Requires 2.7-5.5V external logic power.

More information is available at:

  http://www.superhouse.tv/chairi2c


INSTALLATION
------------
The design is saved as an EAGLE project. EAGLE PCB design software is
available from www.cadsoftusa.com free for non-commercial use. To use
this project download it and place the directory containing these files
into the "eagle" directory on your computer. Then open EAGLE and
navigate to the project.


DISTRIBUTION
------------
The specific terms of distribution of this project are governed by the
license referenced below.


LICENSE
-------
Licensed under the TAPR Open Hardware License (www.tapr.org/OHL).
The "license" folder within this repository also contains a copy of
this license in plain text format.
