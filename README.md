Texas Two Step: A dual axis motor controller for equatorial telescope mounts
============================================================================

![Texas Two Step breadboard prototype controlling an unloaded Orion Astroview]
(https://github.com/dadap/tx2step/raw/master/img/tx2step-breadboard-proto.jpg)

Introduction
------------

The Texas Two Step is a replacement hand controller for the Synta Dual-Axis
motor kit for EQ-3 and EQ-5 type equatorial mounts (it can probably be adapted
to other mount designs utilizing stepper motors as well), with the following
improvements:

* Faster setting rates (original HC is limited to 8x sidereal)
* Analog 2-axis joystick to control setting rates (instead of rate slider switch
  combined with directional pad)
* ST-4 support with independent guiding (ST-4) and setting (joystick) rates.
* Main unit attaches to mount; RA, DEC, ST-4 cables move with telescope, and a
  single cable connects the control unit to a handheld joystick unit.
* Integrated rechargeable battery (to replace 4x D-cell pack, and eliminate the
  additional cable for power)
* All LEDs are dim and red to preserve dark adaptation
* Additional tracking rates (solar, lunar)
* Smaller and lighter
* Lower power draw (current prototype draws about .5 W for tracking)

The Texas Two Step is powered by an Atmel AVR microprocessor (currently an
Atmega328P on a breadboarded prototype circuit) and the stepper motors are
driven by dual TI DRV8834 low-voltage stepper driver ICs. The DRV8834 is
selected to allow a common power source (5V, boosted up from the output of a
single 18650 LiIon cell) for the logic and the motors.

TODO
----

A schematic and PCB layout should eventually be added to this repository.

The following features are not yet implemented:

* Directional switching not implemented; RA tracking direction is dependent on
  the order of stepper motor wiring to the driver boards. The final design is
  intended to allow switching between northern and southern hemisphere operation
  via a DIP switch or jumper setting.

The following features may or may not eventually be implemented:

* Backlash compensation (probably in the declination axis only; RA guiding at
  less than 1x tracking rate requires no backlash compensation.)
* Periodic error correction: implementing this will probably require some form
  of persistent storage, as the available EEPROM storage on the microcontroller
  is likely insufficient.
