# Flight Code

## Info

F.MODE switch controls (right switch)

Switch position   | drone action
 ---------------- | --------------------------------------
DOWN              | manual control
MIDDLE            | altitude hold and heading hold
UP                | autonomous takeoff, then altitude hold


----------------------------------------------------------

Scalings and offsets for Accelerometer

Accel Offsets  |  X:0.04854903 	|  Y: 0.14138588  |  Z: 1.95303880
Accel Scale    |  X:0.99788916 	|  Y: 0.99240148  |  Z: 0.98475480
Gyro Offsets   |  X:0.00221047 	|  Y:-0.02057625  |  Z:-0.02332592



## Changelog

------------------------------------------


April 13, 2015

- Added functionality to land
- Tested. Works.

------------------------------------------


April 14, 2015

- Bearing function is operational
- Made drone coordinates a global struction (drone.lat & drone.lng)

TODO:
- Dig into why it oscilates at some heading angles.

------------------------------------------


April 6, 2015

- Built in heading tracking function. Untested
- CAUTION: flight code is untested...

------------------------------------------


April 3, 2015

- GPS tracking doesn't work because drone pitches back & rolls left too much
- Outdoors, SW_Cal difference of about 100 didn't make much of a difference on stability (cal to 1460 then 1550)

- If drone isnt stable when rcpit & rcroll == 0, that implies an imperfection in the feedback control (or sensors)

------------------------------------------


April 1, 2015 

- Made the mapping for battery voltage and alt hold static to the input of Static_HOVER_THR
- Added constraint to adjustHoverThrottle method 
- Tuning improvements to ESC Calibration; more in recalibration.txt

------------------------------------------


March 2, 2015 (Morning)

- Added a PRINT_DEBUG define so we can easily turn print verbosity ON or OFF with one switch.

- Modifications to Heading_hold, namely:
- Set the compass *inclination offset (this is the offset from magnetic north. At our location on earth it is a vector that points north and about 67deg into the earth. I got the measurement from the MissionPlanner software after an outdoor calibration)
- Added compass.accumulate(); which accumulates readings from compass
- Added null_offsets(): which attempts to maintain the offset that was set at init

- Result: after a quick test this seems to provide more accurate headings.


------------------------------------------

February 26, 2015

- Adjust hover throttle some more
- Tuned altitude hold, works pretty well now


------------------------------------------

February 21, 2015

- Adjusted hover throttle to account for weight
- Brought ALT_STAB kI up to 0.4


------------------------------------------

February 20, 2015

- Changed messaging UARTC port send and receive buffers from 16 bytes to 32 bytes
- Changed the frequency of sending to the phone from once every two seconds to once every second
- Added sendDroneLat, sendDroneLon, sendGPSStatus and sendClimbRate in the sendDataToPhone function
- Disabled GPS lock requirement on startup


------------------------------------------

February 18, 2015

- Added hover throttle adjustment based on battery level
- Heading hold is now in effect when in either autonomous altitude hold or autonomous takeoff
- PIDs updated to Andrew's latest from feature/tracking branch
- GPS update bug fixed in getTakeoffCoordinates()


------------------------------------------

February 16, 2015

- Merged feature/heading back into master


------------------------------------------

February 14, 2015

- Improved upon GPS tracking software using yaw rotation matrix method (untested)
- tuned Altitude hold some more

TODO:

- Test GPS hold (GPS in gym isnt great)
- Add filtering to GPS signal


------------------------------------------

February 13, 2015

- Fixed Heading Control spin issue (tested in Hangar)
- Added GPS lock software using rotation  matrix method

TODO:

- Test GPS hold
- Set drone GPS coordinate as "target" for position hold


------------------------------------------

February 12, 2015

- In manual mode, drone behaves oddly when throttle is all the way down. A quick yaw correction wil fix this temporarily.
- Fixed F.mode channel bug.
- Merged v3_1_7 working takeoff code into master flight.ino.
- Working heading control function with exception of occasional spin.

ISSUE:

- Drone does a full spin ocasinally.

TODO:

- Fix spinning heading issue. 

------------------------------------------

February 11, 2015

- Branched to feature/heading from master
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