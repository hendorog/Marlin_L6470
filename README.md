Marlin_L6470
============

Reprap firmware based on Marlin with added support for L6470 based stepper controllers 

The L6470 driver is based on the Sparkfun sample code, but has been modified to support addressing multiple devices.
Both making a single write to a particular device, and writing to all devices at once are supported.

Current known limitations/opportunities:
* Multimotor suppport only implemented for some commands (GetParam, SetParam, Move)
* Arc support not optimised - the L6470 forces all position moves to complete to zero velocity before the next one can be sent. The result is small blobs at each vertex.
* Strange bug on startup which requires resets and disconnect/reconnect before the x-axis works correctly.