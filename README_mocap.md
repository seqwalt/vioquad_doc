## Things to check when starting Motive:
1. Make sure cameras have solid blue light. Else, enable all cameras in Devices > Tracking > Enable.
2. Devices > Camera Frame Rate set at 100 Hz.
3. Ensure under "streaming" (looks like a wifi symbol) make sure the Local Interface IP is 128.XXX.XXX.XX when using crazyflie (college of engineering). The local ARC lab network is the 192.XXX.X.XXX.


## Mocap Calibration:
1. Open Camera Calibration panel (looks like magic wand)
2. Make sure calibration wand is hidden, and then within Calibration Tab, select mask visible.
3. Select Start Wanding and make sure to get roughly 10,000 data points per camera, and ensureeach camera FOV is filled.
4. Click calculate, then apply calculation.
5. Click Ground Plane tab. Set ground plane square at the origin, and make it level.
6. In Perspective view, highlight the markers corresponding to the ground plane square.
7. Click Set Ground Plane.
8. Export calibration data (.cal file) in directory "This PC"/Documents/Optitrack/"Session 2023-01-24", for example
