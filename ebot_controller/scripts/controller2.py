#!/usr/bin/env python


import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
pose = [0, 0, 0]
Kp1 = .5
Kp2 = 0.8
a = 0


def Waypoints(t):
    x = 2*math.pi*t
    y = 2*math.sin(x)*math.sin(x/2)
    print x
    print y
    return [x, y]


def odom_callback(data):
    global pose
    x1 = data.pose.pose.orientation.x
    y1 = data.pose.pose.orientation.y
    z1 = data.pose.pose.orientation.z
    w1 = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x1, y1, z1, w1])[2]]
    #print pose[0]
    #print pose[2]
    #print pose[2]

# def laser_callback(msg):
#    global regions
 #   regions = {
  #      'bright':  	,
   #     'fright': 	,
    # 'front': min(min(msg.ranges[300:420]), range_max)	,
    #   'fleft':  	,
    #  'bleft':   	,
   # }


def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   # rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
        global pose, a
        a = a + .05
        print ("a =", a)
        c = Waypoints(a)
        print c
        l = c[1]-pose[1]
        m = c[0]-pose[0]
        theta_goal = math.atan2(l, m)
        print theta_goal
        e_theta = theta_goal - pose[2]
        print("e_theta =", e_theta)
        #
        # Your algorithm to complete the obstacle course
        #
        while(pose[0] < c[0]):
            print pose[0], "x position-"
            print c[0], "desired position-"
            if (1.4 < pose[0] < 2.4 or 3.7 < pose[0] < 5.0):
                velocity_msg.linear.x = .05
                velocity_msg.angular.z = Kp1*e_theta
                pub.publish(velocity_msg)
            else:
                velocity_msg.linear.x = .2
                velocity_msg.angular.z = Kp2*e_theta
                pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

