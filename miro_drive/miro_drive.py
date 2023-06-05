#!/usr/bin/env python

import rospy, time
from sensor_msgs import LaserScan
from xycar_msgs.msg import xycar_motor

motor_msg = xycar_motor()
distance = []

def callback(data):
    global distance, motor_msg
    distance = data.ranges
    
def drvie_go():
    global motor_msg
    motor_msg.speed = 20
    motor_msg.angle = 0
    pub.publish(motor_msg)
    
def drive_left():
    global motor_msg
    motor_msg.speed = 10
    motor_msg.angle = -15
    pub.publish(motor_msg)
    
def drive_right():
    global motor_msg
    motor_msg.speed = 10
    motor_msg.angle = 15
    pub.publish(motor_msg)
    
def drive_stop():
    global motor_msg
    motor_msg,speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)
    
rospy.init_node("miro_drive")
rospy.Subscriber("/scan", LaserScan, callback, queue_size = 1)
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size = 1)
time.sleep(3) #ready to connect lidar
while not rospy.is_shutdown():
    ok = 0
    
    for degree in range(0,360):
        
        # front
        for degree in range(79,102):
            if distance[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_stop()
                break
        # left
        for degree in range(31,54):
            if distance[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_left()
                break 
        #front_left
        for degree in range(55,78):
            if distance[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_right()
                break 
        # front_right
        for degree in range(103,126):
            if distance[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_left()
                break 
        # right
        for degree in range(127,150):
            if distance[degree] <= 0.3:
                ok += 1
            if ok >3:
                drive_right()
                break 
            
        if ok <= 3:
            drvie_go 

