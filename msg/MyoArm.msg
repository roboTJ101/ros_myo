#Arm message for the Thalmic Myo. Using firmware 1.1.4.2, 
#Thalmic Gesture Recognition is possible after sync gesture is performed.

#arm represents which arm the device is on
#Enumerated: UNKNOWN=0, RIGHT=1, LEFT=2
uint8 arm
#xdir represents the direction of the x-axis
#Enumerated: UNKNOWN=0, X_TOWARD_WRIST=1, X_TOWARD_ELBOW=2
uint8 xdir
