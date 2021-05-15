#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right',
}
def clbk_laser(msg):
    global regions_
    # print("a")
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), msg.range_max),
        'fright': min(min(msg.ranges[144:287]),msg.range_max ),
        'front':  min(min(msg.ranges[288:431]),msg.range_max ),
        'fleft':  min(min(msg.ranges[432:575]),msg.range_max ),
        'left':   min(min(msg.ranges[576:719]),msg.range_max ),
    }

    take_action()


def change_state(state):
    # print("b")
    global state_, state_dict_
    # if state is not state_:
    print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
    state_ = state

def take_action():
    # print("c")
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1
    
    if regions['left'] > d and regions['fleft'] > d and regions['front'] > d and regions['fright'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] > d and regions['fright'] > d and regions['right'] < d:
        change_state(2)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] > d and regions['fright'] < d and regions['right'] > d:
        change_state(2)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] > d and regions['fright'] < d and regions['right'] < d:
        change_state(2)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] < d and regions['fright'] > d and regions['right'] > d:
        change_state(1)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] < d and regions['fright'] < d and regions['right'] > d:
        change_state(1)
    elif regions['left'] > d and regions['fleft'] > d and regions['front'] < d and regions['fright'] < d and regions['right'] < d:
        change_state(1)
    
    
    
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    # print("d")
def find_wall():
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = -0.5
    # print("e")
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.5
    # print("f")
    return msg
def turn_right():
    msg = Twist()
    msg.angular.z = -0.5
    # print("f")
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg
    # print("g")
def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    # print("g")
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()