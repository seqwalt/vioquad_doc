## Things to check when starting Motive:
1. Make sure cameras have solid blue light. Else, enable all cameras in Devices > Tracking > Enable.
2. Devices > Camera Frame Rate set at 100 Hz.
3. Ensure under "streaming" (looks like a wifi symbol) make sure the Local Interface IP is 128.XXX.XXX.XX when using crazyflie (college of engineering). The local ARC lab network is the 192.XXX.X.XXX.
4. If upon opening Motive the cameras are all in a line on the gound plane, go to File, then select the most recent Calibration Result (toward bootom of pane). If it is not available, a calibration may need to be done (see next section).


## Mocap Calibration:
1. Open Camera Calibration panel (looks like magic wand)
2. Make sure calibration wand is hidden, and then within Calibration Tab, select mask visible.
3. Select Start Wanding and make sure to get roughly 10,000 data points per camera, and ensureeach camera FOV is filled.
4. Click calculate, then apply calculation.
5. Click Ground Plane tab. Set ground plane square at the origin, and make it level.
6. In Perspective view, highlight the markers corresponding to the ground plane square.
7. Click Set Ground Plane.
8. Export calibration data (.cal file) in directory "This PC"/Documents/Optitrack/"Session 2023-01-24", for example

## Tips for good camera placement
https://docs.optitrack.com/hardware/camera-placement?title=Camera_Placement

## Accessing Motive Camera Locations
The Motive API will need to be used in order to obtain the camera locations. The following may be helpful:
- https://forums.naturalpoint.com/viewtopic.php?f=69&t=17247
- https://v21.wiki.optitrack.com/index.php?title=Motive_API
- https://v22.wiki.optitrack.com/index.php?title=Motive_Basics#Calibration_files_.28CAL.29
