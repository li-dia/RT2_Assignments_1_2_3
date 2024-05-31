# RT-Assignment2-Lidia-Loubar
## Description:

This package contains three ROS nodes that collectively enable a robot to receive user-defined goals, navigate to the targets, and provide information about its position and velocity.

## Installation

1. Ensure that you have set up your ROS workspace correctly.

   ```bash
   cd path/to/your/ros_workspace
   catkin_make
   source devel/setup.bash
   ```

2. Clone the Repository:

   ```bash
   git clone https://github.com/li-dia/RT-Assignment2-Lidia-Loubar.git
   ```


## Nodes

### 1. Action Client Node: `action_client_node.py`

- This node acts as an action client to the `/reaching_goal` action server.
- Users can set new target positions or cancel the current goal through user input.
- The node subscribes to the `/odom` topic to receive Odometry messages, extracts relevant information, and publishes a `CustomMessage` on the `/custom_topic` topic.
- Progress feedback is displayed during goal execution.

  #### Pseudo Code:

```python
# Pseudo code for action_client_node.py

# Import necessary ROS and actionlib packages

# 1. Initialize the ROS node.
rospy.init_node('action_client_node')

# 2. Subscribe to the `/odom` topic for robot position information.
rospy.Subscriber('/odom', Odometry, odom_callback)

# 3. Create a publisher for the `CustomMessage` on the `/custom_topic` topic.
custom_pub = rospy.Publisher('/custom_topic', CustomMessage, queue_size=10)

# 4. Prompt the user for a new target or cancel the current goal:
while not rospy.is_shutdown():
    choice = input("Enter 't' for a new target or 'c' to cancel: ")

    if choice.lower() == 't':
        # i. Get target coordinates from the user.
        x = float(input("Enter target x position: "))
        y = float(input("Enter target y position: "))
        # ii. Create an action client for `/reaching_goal`.
        client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        client.wait_for_server()
        # iii. Send the goal and wait for completion.
        set_goal(x, y)
        # iv. Log success or failure of reaching the target.
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Target reached successfully!")
        else:
            rospy.logwarn("Failed to reach the target.")

    elif choice.lower() == 'c':
        # i. Create an action client and cancel the current goal.
        client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        client.wait_for_server()
        client.cancel_goal()

# 5. Define a callback for Odometry messages:
def odom_callback(odom_msg):
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    vel_x = odom_msg.twist.twist.linear.x
    vel_z = odom_msg.twist.twist.angular.z

    custom_msg = CustomMessage()
    custom_msg.x = x
    custom_msg.y = y
    custom_msg.vel_x = vel_x
    custom_msg.vel_z = vel_z

    custom_pub.publish(custom_msg)

# 6. Start the main loop to keep the node running.
rospy.spin()

# End of Pseudo Code
```
### 2. Last Target Service Node: `last_target_service.py`

- Implements a service client to retrieve the last target coordinates from the `/get_last_target_coordinates` service.
- Subscribes to the `/reaching_goal` topic to receive the last target position.

### 3. Robot Position and Velocity Service Node: `robot_pos_vel_service_node.py`

- Provides a service (`/get_robot_pos_vel`) that responds with information about the robot's position and velocity.
- Subscribes to the `/custom_topic` topic, processes `CustomMessage` data, and updates the robot's position and speed.
- Calculates the distance to the last target and the average speed since the last request.

## Usage

1. Run the ROS Nodes using the Launch File:

   Execute the following command to launch the nodes (excluding `action_client_node.py`) in separate terminals:

   ```bash
   roslaunch assignment_2_2023 assignment1.launch
  
2. Run action_client_node.py in a Separate Terminal:
 This node is run separately to isolate its output, avoiding interference with other nodes' outputs. This separation provides a cleaner console view, especially when running multiple nodes simultaneously.
   ```bash
   rosrun assignment_2_2023 action_client_node.py
## Retrieve Robot Information:
- Use the `/get_robot_pos_vel` service provided by `robot_pos_vel_service_node.py` to get information about the robot's position and velocity.
```bash
rosservice call /get_robot_pos_vel
```
Use the  `/get_last_target_coordinates` service provided by `last_target_service.py` to retrieve the last target coordinates.

```bash
rosservice call /get_last_target_coordinates
```
