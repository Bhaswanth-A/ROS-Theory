# 6. ROS Actions
## 6.1 Introduction
Actions are asynchronous calls in ROS, which means that you do not have to wait for a particular task to be finished and you can also do other tasks simultaneously.
Reference: http://wiki.ros.org/actionlib
> ##  What is the difference between Actions and Services?
>
>Actions are asynchronous calls which would perform almost the same functions as Services (kinda), but additionally allows you to work on other tasks in the node (i.e. no blocking code). It also allows you to get feedback, status and even cancel a call.
>
>You can better understand this with an analogy, as explained in this [link](https://www.theconstructsim.com/ros-5-mins-034-ros-action/#:~:text=An%20action%20is%20an%20asynchronous,Action%20Server%20(Pizza%20shop).).
>
>In some cases, if a service takes a long time to execute, the user might want the ability to cancel the request during execution or get periodic feedback about how the request is progressing. The **actionlib** package provides tools to create servers that execute long-running goals that can be preempted. It also provides a client interface in order to send requests to the server.
> A more detaied description: http://wiki.ros.org/actionlib/DetailedDescription
The ActionClient and ActionServer communicate via a "ROS Action Protocol", which is built on top of **ROS messages**. The client and server then provide a simple API for users to request goals (on the client side) or to execute goals (on the server side) via function calls and callbacks.
![](/Images/actions_1.png)
There are 5 Topics provided by an Action Server:
1. **goal**: User (or Client) sends a goal to the Server to initiate the action.
2. **cancel**: User sends a signal under cancel topic to interrupt or stop an action from being executed.
3. **status**: Tells the current status of the server. There are 10 status states - PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, REJECTED, PREEMPTING, RECALLING, RECALLED and LOST.
4. **result**: Provides the final output after executing the action.
5. **feedback**: Provides us with intermediate results about the action while it is being executed.
>**NOTE** - We cannot execute two actions at the same time. Doing so will cancel the previous action from being executed. If a new goal is sent to an action server that is already busy processing a goal, then the currently active goal will be pre-empted. However, this is not a hard limitation as we can have an action server that can process multiple goals.
## 6.2 Action files
The Action definition files have an extension of `.action` and is used to specify the format of _goal_, _result_, _feedback_ message. These files are present in the **action** directory of a package. Here's how a typical action file is defined
```BASH
int32 goal
---
int32 result
---
int32 feedback
```
![](/Images/act_files.png)
In order for the service files to run, a few changes have to be made in the *package.xml* file and *CMakeLists.txt* file.
**In *package.xml***
Modify the file to make the following changes, if not done earlier
```html
<buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>actionlib</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>actionlib</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>actionlib</exec_depend>
  <build_depend>message_generation</build_depend> 
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
```
**In *CMakeLists.txt***
Modify the file to make the following changes, if not done earlier
```py
# Modify the existing line
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)
```
```py
# Generate actions in the 'action' folder
add_action_files(
  FILES
  my_action.action
)
```
```py
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
```
```py
# Add actionlib_msgs as a catkin dependency
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
  actionlib_msgs
)
```
After the necessary changes are made, we need to run `catkin_make` or `catkin_make --only_pkg_with_deps <package_name>` to compile the workspace or a particular package.
After compiling a package, ROS wil create additional messages for you. For example, if your action definition file is named robot.action, then the following messages will be created:
- robotAction.msg
- robotActionGoal.msg
- robotActionResult.msg
- robotActionFeedback.msg
- robotGoal.msg
- robotResult.msg
- robotFeedback.msg
These messages are used by actionlib to facilitate communication between ActionServer and ActionClient.
**The topics used in actions (mentioned above) will use one of these messages, whichever suits its function.**
> In general, if you’re using the libraries in the actionlib package, you should not need to access the autogenerated messages with Action in their type name. The bare Goal, Result, and Feedback messages should suffice. The others are used internally by ROS, i.e., by the actionlib package.

Let us create a custom action file and call it **my_act.action**.
```
# Goal
float32 threshold 
---
# Result
float32 result
---
# Feedback
float32 feed
```
![](/Images/act_files_2.png)
## 6.3 Creating a simple Server
**Code:**
```python
#! /usr/bin/python2
# import actionlib library used for calling actions
import actionlib
import rospy
# import custom action file. Since actions are based on messages, notice how we import actions from the msg directory of a package.
from beginner_tutorials.msg import my_actAction, my_actResult
class Server():
    def __init__(self):
        # create a simple ActionServer by passing the name, action type, and callback function as its parameters. 
        # auto_start has to be declared explicitly and always has to be set to False to prevent autostarting the server (can break your code otherwise).
        self.my_server = actionlib.SimpleActionServer('simple_server', my_actAction, self.execute, auto_start = False)
        # start server
        self.my_server.start()
    def execute(self, action):
        self.res = my_actResult()
        self.res.result = action.threshold
        print("Success")
        print(action.threshold)  
        
        # Some condition for feedback
        # feedback = my_actFeedback()
        # feedback.feed = 10.0 (some float)
        # server.publish_feedback(feedback)
	
	# Once the necessary function is executed, the server notifies the client that the goal is complete by calling set_succeeded.
        self.my_server.set_succeeded(self.res)
        
    
rospy.init_node('Action_Server')
# initialize object called server to call the Server() class.
server = Server()
rospy.spin()
```
**Execution:**
`rosrun <package_name> <file_name>`
Send goals via terminal:
`rostopic pub /action_server_name/goal /package_name/action_message_type parameters`
![](/Images/act_server.png)
## 6.4 Creating a simple Client
**Code:**
```python
#! /usr/bin/python2
import rospy
# import actionlib library
import actionlib
# import custom action file
from beginner_tutorials.msg import my_actAction, my_actGoal, my_actResult, my_actFeedback
# initialize node
rospy.init_node('My_Client')
# create client and specify the name of server and action message type 
client = actionlib.SimpleActionClient('simple_server', my_actAction)
# wait for server (name specified above)
client.wait_for_server()
# def feedback_func(feedback):
    # Do some stuff
# create object of class my_actGoal()
action = my_actGoal()
action.threshold = 55.0
# send goal to server
client.send_goal(action) # client.send_goal(action, feedback_cb=feedback_func)
r = rospy.Rate(1) #1 Hz
while not rospy.is_shutdown():
	# Have some condition to print status
    	# status = client.get_state()
    	# print(status)
	
	# wait for result
	client.wait_for_result()
	# sleep
	r.sleep()
```
### 6.4.1 Explanation
`client.wait_for_result()`: This will wait until the action is complete and blocks the remaining code. This won't allow you to continue to work on your thread.
`client.get_state()`: It returns an integer that specifies the state of the action. There are 10 possible states, a few of which are
- 0 implies PENDING
- 1 implies ACTIVE
- 2 implies PREEMPTED
- 3 implies SUCCEEDED
- 4 implies ABORTED
If your get_state() is less than 2, it indicates that your action is still not complete.
`client.cancel_goal()`: Used to cancel or preempt a goal.
**Execution**

![](/Images/act_client.png)

## Turtlesim example

As you can see below, running the following commands instructs turtlesim to move along a pentagon.

![](/Images/act_tur_1.png)

On running `rostopic list` we see a list of all the current topics. `/turtle_shape/goal` is one such topic, which allows you to publish messages to the server through the terminal. It allows us to enter the number of edges and define the radius as well.

![](/Images/act_tur_2.png)

### GUI

The actionlib offers a graphical way to send goal to action server.
Syntax-
`rosrun actionlib axclient.py /name_of_the_action`

The GUI has areas for **Goal**, **Feedback** and **Result**. You can press the **SEND GOAL** button to send goals with the relevant parameters, and you can also cancel or preempt a goal anytime with the **CANCEL GOAL** button. After the action finished successfully, the GUI shows the **Result**.

![](/Images/act_tur_3.png)
