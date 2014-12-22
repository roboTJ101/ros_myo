#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import EmgArray

if __name__ == '__main__':

    rospy.init_node('turtlesim_driver', anonymous=True)
    
    # Publish to the turtlesim movement topic
    tsPub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    def strength(emgArr1):
	emgArr=emgArr1.data
	# Define proportional control constant:
	K = 0.005
	# Get the average muscle activation of the left, right, and all sides
	aveRight=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
	aveLeft=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
	ave=(aveLeft+aveRight)/2
	# If all muscles activated, drive forward exponentially
	if ave > 500:
	    tsPub.publish(Twist(Vector3(0.1*math.exp(K*ave),0,0),Vector3(0,0,0)))
	# If only left muscles activated, rotate proportionally
	elif aveLeft > (aveRight + 200):
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*ave)))
	# If only right muscles activated, rotate proportionally
	elif aveRight > (aveLeft + 200):
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*ave)))	
	# If not very activated, don't move (high-pass filter)    
	else: 
	    tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

    rospy.Subscriber("myo_emg", EmgArray, strength)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
