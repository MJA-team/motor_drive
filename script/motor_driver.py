#!/usr/bin/env python  

import rospy
import math
import geometry_msgs.msg

max_vel = 4.0
max_pwm = 1000.0

def scaling(input,in_max,out_max):
    out_t = input / in_max * out_max
    if out_t > out_max:
        output = out_max
    else:
        output = out_t
    return output

def callback(data):    
    lin_vel = scaling(data.linear.x, max_vel, max_pwm)
    ang_vel = scaling(data.angular.z, max_vel, max_pwm)
    rospy.loginfo(rospy.get_caller_id() + " value linear x: %s; value angular z: %s", lin_vel, ang_vel)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver', anonymous=True)

    rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':    
    while not rospy.is_shutdown():
        listener()
