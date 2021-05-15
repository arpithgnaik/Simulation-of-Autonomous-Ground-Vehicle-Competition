#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from std_msgs.msg import String

def movebase_client(x, y):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def callback():
            result = True
            for i in range(4):
                result = result and movebase_client(x[i],y[i])
                # pub.publish(hello)
                time.sleep(5)
            if result:
                rospy.loginfo("Goal execution done!")


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        # x = [10.7, 12.4,18.2, -2.0]
        # y = [10.5, -1.4, -1.4, 4.0]
        # hello="got"
        x=[7.67]
        y=[1.6]

        # pub = rospy.Publisher('my_topic', String, queue_size=10)
        # sub=rospy.Subscriber("topic", String, callback)

        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

# drop1 pose=6.881193   2.660478