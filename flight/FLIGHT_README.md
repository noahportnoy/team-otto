# Flight Code

## Info

**WARNING: AUTONOMOUS TAKEOFF NEEDS FURTHER TESTING**

F.MODE switch controls (right switch)

Switch position   | drone action
 ---------------- | --------------------------------------
DOWN              | manual control
MIDDLE            | altitude hold and heading hold
UP                | autonomous takeoff, then altitude hold

*Note: whenever switch is put in the UP position, drone will attempt TAKEOFF first, followed by HOLD.*




## Changelog


February 11, 2015

- Using AHRS for heading control. 
- Made some tuning changes to the pitch / roll PID's for more stable flight
- Increased the hover throttle because of the added weight that came with the new Hardware (3rd level)

ISSUE:
	- Drone will only hold heading for a shirt period of time, then it will start to spin.
TODO:
	- Fix spinning heading issue. 

------------------------------------------

February 7, 2015

- Changed F.MODE switch position mapping to allow more states. Simply keep the safety on and select the mode you’d like to be in.  
- Added UART messaging so that alt & battery data is sent through the RPi to app.  
- Added a heading control function. Seems to be jerky, I’m going to try smoothing out the input.  
- Fixed roll, pitch, and yaw values. Now we are getting values from the DMP (was commented out before). As a result I tuned the flight stabilization PIDs some more.  
- Flies well in manual mode. Alt_hold needs more tuning.
- Branched from v3_1_7  

------------------------------------------


February 6, 2015

- Minor reformatting of code. Compiled successfully but should be tested.  

------------------------------------------


February 5, 2015

- Established behavioral parity with v3_1_6 by means of thorough testing  
- Major code reorganization  
- Branched from 3_1_6  
- Tests in office promising for autonomous takeoff. Tests in boyden needed to confirm.  

------------------------------------------


February 4, 2015

- Changing from map(……) function to adding ~1290. This is the approximate throttle that the drone will hover at.  
- Working altitude hold function!  

-----------------------------------------