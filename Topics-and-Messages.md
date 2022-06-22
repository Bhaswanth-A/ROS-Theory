# 3. ROS Topics

ROS Topics represent a stream of messages that connect two or more nodes. The nodes act as publishers or subscribers of messages - publisher nodes send messages through topics and subscriber nodes receive the messages transmitted through a particular topic. 

- A publisher node would have to register the topic name and the type of messages, and only then it can publish to a topic. 

- A subscriber node would make a request to roscore to get details about a topic in order to receive the transmitted messages sent through it.

>In ROS, all messages on the same topic must have the same data type.
>Any number of nodes can publish to a topic as long as they have the same message type.

>A ROS Topic cannot be published without initializing a ROS Node.

## 3.1 Using rostopic

The rostopic command is used to display information about ROS topics that are currently running.

![](/Images/topic_1.png)

![](/Images/topic_2.png)

![](/Images/topic_3.png)

![](/Images/topic_4.png)

rostopic hz reports the rate at which data is being published. The below image tells us that turtlesim is publishing data about our turtle at the rate of 60 Hz.

![](/Images/topic_5.png)

# 4. ROS Messages

## 4.1 ROS msg

The message definition file has an extension of `.msg` and all such files need to be located inside the `msg` directory of a package.

## 4.2 Creating a ROS msg

![](/Images/msg_1.png)

Now we need to make sure that the msg files are turned into source code for Python, C++, and other languages.
Open *package.xml* and make sure the following lines are uncommented.

```html
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```
Now we need to make changes to the *CMakeLists.txt* file.

```py
# Modify the existing text
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
```py
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
```py
add_message_files(
  FILES
  Message1.msg
  Message2.msg
)
```
```py
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
Now since we have made a few changes to our package and created new files/directories, we need to compile out workspace by running `catkin_make` or `catkin_make --only-pkg-with-deps <package_name>`.


## 4.3 Using rosmsg

Documentation - http://wiki.ros.org/rosmsg

![](/Images/msg_2.png)

![](/Images/msg_3.png)


## 4.4 Publishing to a Topic

```python
#! /usr/bin/python2

import rospy
from std_msgs.msg import String
import time

rospy.init_node('Node_name',anonymous=True)

pub = rospy.Publisher('/orchis_dark', String, latch=True, queue_size=10)

r = rospy.Rate(10)

while True:
	pub.publish('Current time: {}'.format(time.time()))
	r.sleep()
```
### 4.4.1 Code explanation:

Line 1: `#! /usr/bin/python2` is known as the shebang. It lets the kernel know that this is a Python file and that it should be passed into the Python interpreter.

Line 3: `import rospy` appears in every ROS node and imports some basic functionalities, classes and functions.

Line 4: this line allows us to reuse the `std_msgs/String` message type for publishing.

![](/Images/pub_1.png)

Line 8: This is used to initialize a ROS node and the node takes the name that you give it. anonymous = True ensures that you have a unique name by adding random numbers to the end of name.

Line 9: pub here is an object of the class Publisher, and allows us to publish to any topic of any message type that you give it. queue_size limits the amount of queued messages if any subscriber is not receiving them fast enough.

Line 11: This line creates an object r of the class Rate. When r.sleep() is called, it sleeps just long enough for the loop to run at the desired rate. When the argument 10 is passed, it goes through the loop 10 times per second.

Line 14: This is used to publish the desired message to the topic.

![](/Images/pub_2.png)

![](/Images/pub_3.png)

rostopic hz tells us the rate at which messages are being published. Here we see that it is being published at a rate specified by the argument in rospy.Rate(arg).

![](/Images/pub_4.png)

## 4.5 Subscribing to a Topic

```python
#! /usr/bin/python2

import rospy
from std_msgs.msg import String

rospy.init_node('simple_subscriber', anonymous=True)

def function(my_string):
	print(my_string.data)
	
rospy.Subscriber('/force', String, function)

rospy.spin()
```

### 4.5.1 Code explanation:

Line 1: `#! /usr/bin/python2` is known as the shebang. It lets the kernel know that this is a Python file and that it should be passed into the Python interpreter.

Line 3: `import rospy` appears in every ROS node and imports some basic functionalities, classes and functions.

Line 4: this line allows us to reuse the `std_msgs/String` message type for publishing.

![](/Images/sub_1.png)

Line 6: This is used to initialize a ROS node and the node takes the name that you give it. anonymous = True ensures that you have a unique name by adding random numbers to the end of name.

Line 8-9: This is a callback function. Once a node has subscribed to a topic, everytime a message arrives on it, the associated callback function is called with the message as it’s parameter.

Line 11: In this line, we subscribe to the topic, giving it the name of the topic, message type and callback function as its arguments.

Line 13: This basically instructs ROS to loop over again, once a subscription has been made.

Behind the scenes, the subscriber passes this information on to roscore and tries to make a direct connection with the publishers of this topic. If the topic does not exist, or if the type is wrong, there are no error messages: the node will simply wait until messages start being published on the topic.

> A subscriber callback function is not executed continuously, that is, it is not processing all the time. It will only process when new data is published.

![](/Images/sub_2.png)

![](/Images/sub_3.png)

On VS Code - Output:

![](/Images/sub_4.png)
