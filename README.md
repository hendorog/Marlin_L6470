Marlin_L6470
============

Reprap firmware based on Marlin with added support for L6470 based stepper controllers 

Timer based interrupt for step-dir type stepper drivers is not required and has been disabled.

Theoretically this should run on lower power/speed cpu's. The stepper drivers do motion control which mean the CPU doesn't need to.
However the size of the firmware is still large - around 50K.