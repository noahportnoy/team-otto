Version 3_1_7


WARNING: AUTONOMOUS TAKEOFF NEEDS FURTHER TESTING

Right switch controls:
	DOWN   	: drone off
	MIDDLE 	: autonomous takeoff, then altitude hold
	UP		: manual control

Note: whenever switch is put in the MIDDLE position, drone will attempt TAKEOFF first, followed by HOLD.


Abbreviated changelog
------------------------------------------

February 5, 2015

Tests in office promising for autonomous takeoff. Tests in boyden needed to confirm.

Established behavioral parity with v3_1_6 by means of thorough testing
Major code reorganization
Branched from 3_1_6
------------------------------------------

February 4, 2015

Changing from map(……) function to adding ~1290. This is the approximate throttle that the drone will hover at.
!WORKING ALT HOLD FUNCTION!

TODO:
-Fine tune the PID loop to make more stable.
-----------------------------------------