import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from xycar_motor.msg import xycar_motor
import time
import numpy as np

pub = None

def callback(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    take_action(regions)

def take_action(regions):

    state_description = ''
    global case
    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 1 - nothing'
        case = '1'
        
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        case = '2'
        
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        case = '3'
        
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        case = '4'
        
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        case = '5'
        
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        case = '6'
        
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        case = '7'
        
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        case = '8'
        
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)

def motor(angle, speed):
    global pub
    global motor_control

    motor_control.angle = angle
    motor_control.speed = speed

    pub.publish(motor_control)

motor_control = xycar_motor()

def main():
    
    rospy.init_node("miro_drive")
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

    while not rospy.is_shutdown():
        
        try:
            
            if case == '1':                      # nothing
                angle = 0
                speed = 30
                motor(angle,speed)              
                
            elif case == '2':                    # front
                angle = -5
                speed = 0
                motor(angle,speed)
                
            elif case == '3':                    # fright
                angle = -5
                speed = 0
                motor(angle,speed)
                        
            elif case == '4':                    # fleft
                angle = 5
                speed = 0
                motor(angle,speed)
                
            elif case == '5':                    # front and fright
                angle = -5
                speed = 0
                motor(angle,speed)
                
            elif case == '6':                   # front and fleft
                angle = 5
                speed = 0
                motor(angle,speed)
                
            elif case == '7':                   # front and fleft and fright
                angle = -5
                speed = 0
                motor(angle,speed)
                
            elif case == '8':                   # fleft and fright
                angle = -5
                speed = 0
                motor(angle,speed)
                
        except: pass

if __name__ == '__main__':
    main()