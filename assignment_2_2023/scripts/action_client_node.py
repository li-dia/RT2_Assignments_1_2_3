#!/usr/bin/env python

"""
.. module:: action_client_node
    :platform: Unix
    :synopsis: Python module that implements an action client, allowing the user to set a target (x, y) position for the robot or to cancel it.
    
    The node also publishes the robot's position and velocity on a custom message.

.. moduleauthor:: Lidia Loubar

Subscribes to:
    /odom: Subscribes to the Odometry message to extract the robot's position and velocity.

Publishes to:
    /custom_topic: Publishes the CustomMessage with the robot's position and velocity.

Action:
    /reaching_goal: Action client that sends goal positions to the action server.
    The action is defined in the assignment_2_2023 package.

"""

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, CustomMessage
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

class ActionClientNode:
    """
    Class that implements an action client node to set a target (x, y) position for the robot or to cancel it.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('action_client_node')

        # Create the action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()

        # Initialize goal positions
        self.goal_x = None
        self.goal_y = None

        # Create subscribers and publishers
        self.custom_pub = rospy.Publisher('/custom_topic', CustomMessage, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def set_goal(self, x, y):
        """
        Function to set a goal position for the robot.
        
        :param x: X-coordinate of the goal position.
        :type x: float
        :param y: Y-coordinate of the goal position.
        :type y: float
        """
        # Store the goal position
        self.goal_x = x
        self.goal_y = y

        # Create a goal with the specified target position (x, y)
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Send the goal to the action server with feedback callback
        self.client.send_goal(goal, done_cb=self.goal_done_callback)
        rospy.loginfo("Goal sent: x={}, y={}\n".format(x, y))
        
        
    def goal_done_callback(self, status, result):
        """ 
        Function to handle the goal completion.
        
        :param status: The status of the goal.
        :type status: int
        :param result: The result of the goal.
        :type result: PlanningResult
        """
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded\n")


    def cancel_goal(self):
        """
        Function to cancel the goal.
        """
        # Cancel the goal
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled\n")

    def odom_callback(self, odom_msg):
        """
        Callback function to process the Odometry message and publish the CustomMessage.
        
        :param odom_msg: The received odometry message.
        :type odom_msg: Odometry
        """
        # Process the Odometry message and extract relevant information
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vel_x = odom_msg.twist.twist.linear.x
        vel_z = odom_msg.twist.twist.angular.z

        # Create a CustomMessage with the extracted information
        custom_msg = CustomMessage()
        custom_msg.x = x
        custom_msg.y = y
        custom_msg.vel_x = vel_x
        custom_msg.vel_z = vel_z

        # Publish the CustomMessage on the /custom_topic topic
        self.custom_pub.publish(custom_msg)

    def main_loop(self):
        """
        Main loop to get user input for target position and handle goal setting and cancellation.
        """
        while not rospy.is_shutdown():
            try:
                # Get target position from user input
                x = float(input("Enter target x position: "))
                y = float(input("Enter target y position: "))
                # Call the set_goal function with user-input target position
                self.set_goal(x, y)
                cancel_input = input("Type 'cancel' to cancel immediately or press 'enter' to continue: \n")
                if cancel_input.lower() == 'cancel':
                    # Call the cancel_goal function
                    self.cancel_goal()
                
            except ValueError:
                rospy.logerr("Invalid input. Please enter numerical values.")

if __name__ == "__main__":
    """
    Main function to initialize the ActionClientNode and start the main loop.
    """
    action_client_node = ActionClientNode()
    action_client_node.main_loop()
    rospy.spin()

