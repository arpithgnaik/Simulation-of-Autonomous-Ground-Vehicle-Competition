#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

pose = [0,0,0]
Kp1 = .5
Kp2 = .6
t = 0
state_ = 0
pub = None

regions = {
        'bright': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'bleft': 0,
}

state_dict_ = {
    0: 'move to target',
    1: 'turn left',
    2: 'check wall',
    
}
def Waypoints():
    global pose,t
    t = t+ 0.05
    if (pose[0] < 6.2831853):
       x  = 2*math.pi*t
       y  = 2*math.sin(x)*math.sin(x/2)
       print x
       print y 
       print t
       return [x,y]
    else :
       return [12.5,0]
def odom_callback(data):
    global pose
    x1  = data.pose.pose.orientation.x
    y1  = data.pose.pose.orientation.y
    z1  = data.pose.pose.orientation.z
    w1  = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x1,y1,z1,w1])[2]]
    
def laser_callback(msg):
    global regions
    regions = {
        'bright': min(min(msg.ranges[0:143]), msg.range_max),
        'fright': min(min(msg.ranges[144:287]), msg.range_max),
        'front' : min(min(msg.ranges[288:431]), msg.range_max),
        'fleft' : min(min(msg.ranges[432:575]), msg.range_max),
        'bleft' : min(min(msg.ranges[576:719]), msg.range_max),
    }
    take_action()

def take_action():
    global regions
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 1 - nothing'
        change_state(0)
    
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 3 - fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 4 - fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        # state_description = 'unknown case'
        rospy.loginfo(regions)

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state
    
def error_angle(c):
        global pose
        print c[1],c[0]
	g = math.atan2(c[1]-pose[1],c[0] -pose[0])
        print g-pose[2]
        return (g-pose[2])

def euclidean_distance(c):
	return math.sqrt(pow((c[0]-pose[0]),2)+ pow((c[1]-pose[1]),2))

def turn_left():
    velocity_msg = Twist()
    velocity_msg.linear.x =0.5#0.1
    velocity_msg.angular.z = 0.5
    return velocity_msg

def control_loop():
    global pose,regions
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while (pose[0] < 6.2831853):
       
    	c = Waypoints()
    	e_theta = error_angle(c) 
        print ("its running")
        print c
        
        while(pose[0] < c[0]):            
	     print pose[0],"x position inside loop-"
	     print c[0],"desired position-"
	     print("e_theta =",e_theta)
             print pose[2]
             if ( 1.4 < pose[0] < 2.4 or 3.9 < pose[0] < 4.9):
    	   	velocity_msg.linear.x = .05
       	        velocity_msg.angular.z = Kp1*e_theta
 	        pub.publish(velocity_msg)
       	     elif (-1 < pose[0] < 1.4 or 2.4 < pose[0] < 3.9 or 4.9 < pose[0] < 6.28 ):
       	        velocity_msg.linear.x = .1
       	        velocity_msg.angular.z = Kp2*e_theta
		pub.publish(velocity_msg)
             else :
		break

    while (pose[0] > 6.2831853):
	print ("loop completed and begining")
        velocity_msg = Twist()
        if state_ == 0 :
            c = Waypoints()           #create msg
    	    print c
	    print e_theta
            print pose[0]
            print euclidean_distance(c)
            velocity_msg.linear.x = 0.05*euclidean_distance(c)
       	    velocity_msg.angular.z = .5*error_angle(c) 
                
        elif state_ == 1:
            print("turning left and follow wall")
            velocity_msg = turn_left()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub.publish(velocity_msg)
        
        rate.sleep()
	
		
        	 
    print("Controller message pushed at {}".format(rospy.get_time()))
    rate.sleep()
      
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass