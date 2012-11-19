Marlin_L6470

Reprap firmware based on Marlin with added support for L6470 based stepper controllers 

Timer based interrupt for step-dir type stepper drivers is not required and has been disabled.

Theoretically this should run on lower power/speed cpu's. The stepper drivers do motion control which mean the CPU doesn't need to.
However the size of the firmware is still large - around 50K.
The L6470 driver is based on the Sparkfun sample code, but has been modified to support addressing multiple devices.
Both making a single write to a particular device, and writing to all devices at once are supported.

Current known limitations/opportunities:
* Multimotor suppport only implemented for some commands (GetParam, SetParam, Move)
* Arc support not optimised - the L6470 forces all position moves to complete to zero velocity before the next one can be sent. The result is small blobs at each vertex.
* X and Y axis need to be off the home switch when resetting/powering on
* Code size seems large, most step/dir code has been removed using #defines when using the L6470 vector drivers.
* Very hacky code, code written to make it work, not to make it pretty
* The resolution of the motor velocity parameter in the 6470 Move command is not very high. This causes 'glitches' at the end of long lines. It might be fixable using the Run command instead as it allows the velocity to be specified more accurately.

Currently implementing run_mode, which enables smooth arcs as stopping is avoided.