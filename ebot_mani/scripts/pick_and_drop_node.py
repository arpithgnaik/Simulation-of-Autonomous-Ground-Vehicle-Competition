#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('pick_and_drop_node', anonymous=True)  #initialising node
        #defining objects for class ur5Moveit       
        self._planning_group = "ur5_planning_group"           
        self._planning_group2 = "gripper_planning"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group) #group for planning arm
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)#group2 for planning hand
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction) #action-client for executing trajectory
        self._exectute_trajectory_client.wait_for_server()

        
    def arm_planning(self, arg_pose_name):   #planning function for arm
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m') 
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def hand_planning(self, arg_pose_name):  #planning function for the gripper
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group2.set_named_target(arg_pose_name)
        plan = self._group2.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')    
    

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    
    #picking up of soap 1 and droping in leftbox by going through predefined pose
    ur5.arm_planning("pic_soap")           #soap1 upperpoint
    
    ur5.arm_planning("pic_soap_down")      #soap between the gripper
    rospy.sleep(2)
    ur5.hand_planning("grip_sp")           # gripping
    rospy.sleep(2)
    ur5.arm_planning("db")                 #leftbox pose
    
    ur5.hand_planning("open")              #opening the hand
    

    #picking up of soap 2 and droping in rightbox by going through predefined pose
    ur5.arm_planning("ob2")          #soap2 upperpoint
    
    ur5.arm_planning("ob2_down")      #soap2 between the gripper
   
    ur5.hand_planning("gs1")       # gripping
    rospy.sleep(1)
    ur5.arm_planning("ob2") 
    ur5.arm_planning("db2")          #right box pose
    
    ur5.hand_planning("open")             #opening the hand
    
   
    #picking up of biscuit 1 and droping in rightbox by going through predefined pose
    ur5.arm_planning("pic_bis")         #biscuit1 upperpoint
    
    ur5.arm_planning("new_bis")          #biscuit btw the gripper
    
    ur5.hand_planning("grip_bis")        #gripping biscuit
    rospy.sleep(1)
    ur5.arm_planning("db2")              #right box pose
    
    ur5.hand_planning("open")            #opening the gripper
    

    ur5.hand_planning("close")
    
    del ur5


if __name__ == '__main__':
    main()


