#!/usr/bin/env python

import rospy
import math
import geometry_msgs.msg
import RPi.GPIO as gpio

max_vel = 4.0
max_pwm = 1000.0
in1Left = 7
in2Left = 11
enabLeft = 16
in1Right = 13
in2Right = 15
enabRight = 18
dutyLeft = 0.0
dutyRight = 0.0
velLeft = 0.0
velRight = 0.0

gpio.setmode(gpio.BOARD)
gpio.setup(in1Left, gpio.OUT)
gpio.setup(in2Left, gpio.OUT)
gpio.setup(in1Right, gpio.OUT)
gpio.setup(in2Right, gpio.OUT)
gpio.setup(enabLeft, gpio.OUT)
gpio.setup(enabRight, gpio.OUT)
pLeft=gpio.PWM(enabLeft,max_pwm)
pRight=gpio.PWM(enabRight,max_pwm)


def calcDuty(lin_vel, ang_vel):
    #Initial condition
    dutyLeft = 0
    dutyRight = 0
    velLeft = 0
    velLeft = 0

    if ang_vel == 0:
        velLeft = lin_vel
        velRight = lin_vel
    if ang_vel > 0:
        velLeft = lin_vel + ang_vel
        velRight = lin_vel + (ang_vel * -1)
    if ang_vel < 0:
        velLeft = lin_vel + (ang_vel * -1)
        velRight = lin_vel + ang_vel
    
    dutyLeft = abs(velLeft)
    dutyRight = abs(velRight)

    if dutyLeft > 100:
        dutyLeft = 100
    if dutyRight > 100:
        dutyRight = 100

    pLeft.start(dutyLeft)
    pRight.start(dutyRight)
    pesan = (" calcDuty value velLeft: %s; value velRight: %s; value dutyLeft: %s; value dutyRight: %s /n" % (velLeft, velRight, dutyLeft, dutyRight))
    rospy.loginfo(rospy.get_caller_id() + pesan)
    print(rospy.get_caller_id() + pesan)

def send_raspi(velLeft, velRight):
    if velLeft > 0.0:
        # Ban Left maju
        gpio.output(in1Left,False)
        gpio.output(in2Left,True)
        pesan = " send_raspi Ban Left maju/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)
    if velLeft < 0.0:
        # Ban Left mundur
        gpio.output(in1Left,True)
        gpio.output(in2Left,False)
        pesan = " send_raspi Ban Left mundur/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)
    if velLeft == 0.0:
        # Ban Left stop
        gpio.output(in1Left,False)
        gpio.output(in2Left,False)
        pesan = " send_raspi Ban Left stop/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)
    if velRight > 0.0:
        # Ban Right maju
        gpio.output(in1Right,False)
        gpio.output(in2Right,True)
        pesan = " send_raspi Ban Right maju/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)
    if velRight < 0.0:
        # Ban Right mundur
        gpio.output(in1Right,True)
        gpio.output(in2Right,False)
        pesan = " send_raspi Ban Right mundur/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)
    if velRight == 0.0:
        # Ban Right stop
        gpio.output(in1Right,False)
        gpio.output(in2Right,False)
        pesan = " send_raspi Ban Right stop/n"
        rospy.loginfo(rospy.get_caller_id() + pesan)
        print(rospy.get_caller_id() + pesan)

def scaling(input,in_max,out_max):
    out_t = input / in_max * out_max
    if out_t > out_max:
        output = out_max
    else:
        output = out_t
    return output

def callback(data):    
    lin_vel = scaling(data.linear.x, max_vel, 100.0)
    ang_vel = scaling(data.angular.z, max_vel, 100.0)
    pesan = (" value linear x: %s; value angular z: %s" % (lin_vel, ang_vel))
    rospy.loginfo(rospy.get_caller_id() + pesan)
    print(rospy.get_caller_id() + pesan)
    calcDuty(lin_vel, ang_vel)
    send_raspi(velLeft, velRight)
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_drive', anonymous=True)

    rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def stop_total():
    #Stop
    rospy.loginfo(rospy.get_caller_id() + " stop_total STOP TOTAL")
    gpio.output(in1Left,False)
    gpio.output(in2Left,False)
    gpio.output(in1Right,False)
    gpio.output(in2Right,False)
    pLeft.stop()
    pRight.stop()
    gpio.cleanup()
    print('ROS is STOP TOTAL')

if __name__ == '__main__':    
    while not rospy.is_shutdown():
        print('ROS is running')
        listener()
    stop_total()
