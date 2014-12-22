#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy, math
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import MyoArm, EmgArray

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0 	#
#    RIGHT          = 1		#
#    Left           = 2		#
# MyoArm.xdir___________________#
#    UNKNOWN        = 0		#
#    X_TOWARD_WRIST = 1		#
#    X_TOWARD_ELBOW = 2		#
# myo_gest UInt8________________#
#    REST           = 0		#
#    FIST           = 1		#
#    WAVE_IN        = 2		#
#    WAVE_OUT       = 3		#
#    FINGERS_SPREAD = 4		#
#    THUMB_TO_PINKY = 5		#
#    UNKNOWN        = 255	#
#################################


if __name__ == '__main__':

    global armState
    global xDirState
    armState = 0;

    rospy.init_node('turtlesim_driver', anonymous=True)

    turtlesimPub = rospy.Publisher("directs", String, queue_size=10)
    tsPub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    def setArm(data):
	global armState
	global xDirState

        armState = data.arm
        xDirState = data.xdir

  
    def strength(emgArr1):
	emgArr=emgArr1.data
	K = 0.001
	aveLeft=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
	aveRight=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
	ave=(aveLeft+aveRight)/2
	#rospy.loginfo(aveLeft, aveRight)
	if ave > 500:
	    tsPub.publish(Twist(Vector3(2.0*math.exp(0.005*ave),0,0),Vector3(0,0,0)))
	elif aveLeft > (aveRight + 200):
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*ave)))
	elif aveRight > (aveLeft + 200):
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*ave)))	    
	else: 
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))


    rospy.Subscriber("myo_arm", MyoArm, setArm)
    rospy.Subscriber("myo_emg", EmgArray, strength)
    rospy.loginfo('running!')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
