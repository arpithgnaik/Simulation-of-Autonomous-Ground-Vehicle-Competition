#!/usr/bin/env python

import rospy                                            # importing rospy lib
from geometry_msgs.msg import Twist                     # importing Twist --linear and angular velocity
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math                                             #importing math

pose = [0,0,0]
Kp1 = .6                                                #defining proportional constant
Kp2 = .7                                                #defining proportional constant
Kp3 = 0.05                                              #defining proportional constant
Kp4 = .5                                                #defining proportional constant
t = 0                                                   #defining t variable
state_ = 0                                              #defining state variable
c = [0,0]

regions = {                                             # defining regions dictionary
        'bright': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'bleft': 0,
}

def Waypoints():
    global pose,t
    t = t+ 0.05
    if (pose[0] < 2*math.pi):
       x  = 2*math.pi*t
       y  = 2*math.sin(x)*math.sin(x/2)
       return [x,y]
    else :
       return [12.50,0.0]                                #goal position
def odom_callback(data):
    global pose
    x1  = data.pose.pose.orientation.x
    y1  = data.pose.pose.orientation.y
    z1  = data.pose.pose.orientation.z
    w1  = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x1,y1,z1,w1])[2]]
    
def laser_callback(msg):
    global regions
    #laser readings receives from subscriber topic--/ebot/laser/scan
    #total 720 samples, 5 regions are defined-- 144 samples per side, i.e 54 deg for each side
    regions = {
        'bright': min(min(msg.ranges[0:143]), msg.range_max),                                       #back-right
        'fright': min(min(msg.ranges[144:287]), msg.range_max),                                     #front-right
        'front' : min(min(msg.ranges[288:431]), msg.range_max),                                     #front
        'fleft' : min(min(msg.ranges[432:575]), msg.range_max),                                     #front-left
        'bleft' : min(min(msg.ranges[576:719]), msg.range_max),                                     #back-left
    }
    take_action()

def take_action():
    global regions
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:                                 #case 1 -> if there is no obstacle in front, front-left, front-right 
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:                               #case 2 -> if obstacle at front side
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:                               #case 3 -> if obstacle at front-right
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:                               #case 4 -> if obstacle at front-left
        change_state(1)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:                               #case 5 -> if obstacle at front and front-right
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:                               #case 6 -> if obstacle at front and front-left
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:                               #case 7 -> if obstacle at front, front-left and front-right'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:                               #case 8 -> if obstacle at front-left and front-right
        change_state(0)
    else:
        print ("unknown case")
        rospy.loginfo(regions)

def change_state(state):
    global state_
    if state is not state_:
        print ("changing state")
        state_ = state
    
def error_angle(c):
    global pose
    return (math.atan2(c[1]-pose[1],c[0]-pose[0])-pose[2])

def euclidean_distance(c):
    return math.sqrt(pow((c[0]-pose[0]),2)+ pow((c[1]-pose[1]),2))

def turn_left_and_move():
    velocity_msg = Twist()
    velocity_msg.linear.x =0.07
    velocity_msg.angular.z = 0.5
    return velocity_msg

def control_loop():
    global pose,regions
    rospy.init_node('ebot_controller')                                                                           # Initialize the ebot_controller Node.
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)                                                      # Create a handle to publish message to a topic
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)                                              # Subscribe to the /ebot/laser/scan topic and attach a Callback Funtion to it
    rospy.Subscriber('/odom', Odometry, odom_callback)                                                           # Subscribe to the /odom topic and attach a Callback Funtion to it
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()                                                                                       # defining the vel_msg to Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while (pose[0] < 2*math.pi):
    	c = Waypoints()
        while (pose[0] < c[0]):
            if ( 1.4 < pose[0] < 2.35 or 3.9 < pose[0] < 4.7 ):
                velocity_msg.linear.x = .08
                velocity_msg.angular.z = Kp1*error_angle(c)
                pub.publish(velocity_msg)
            else :
                velocity_msg.linear.x = .12
                velocity_msg.angular.z = Kp2*error_angle(c)
                pub.publish(velocity_msg)
        rate.sleep()    

    while (pose[0] > 2*math.pi):
        if state_ == 0 :
            c = Waypoints()          
    	    print c    
            print pose[0]
            print euclidean_distance(c)
            velocity_msg.linear.x = Kp3*euclidean_distance(c)
       	    velocity_msg.angular.z = Kp4*error_angle(c) 
                
        elif state_ == 1:
            print("turning left and follow wall")
            velocity_msg = turn_left_and_move()     
            pass

        else:
            rospy.logerr('Unknown state!')
        if (euclidean_distance(c) < 0.1):
            break
        pub.publish(velocity_msg)
        rate.sleep()
			
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)    	 
    print("Controller message pushed at {}".format(rospy.get_time()))
    rate.sleep()
      
if __name__ == '__main__':
    try:
        control_loop()#main function
    except rospy.ROSInterruptException:
        pass
