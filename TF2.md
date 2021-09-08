# 7. ROS TF2

## 7.1 Introduction to TF2

**TF2** is the second generation of the *transform library*, which lets the user keep track of multiple coordinate frames over time. **TF2** maintains the relationship between coordinate frames over time, and lets the user transform points, vectors, etc. between any two coordinate frames at any desired point in time.

TF2 is not a centralized service, but is a distributed protocol with which many different robots will communicate about the state of the robot.

In TF2, **Publishers** are known as **Broadcasters**, and **Subscribers** are known as **Listeners**.

- **Listeners** listen to **/tf** and cache all data heard up to the cache limit.
- **Broadcasters** publish transforms between coordinate frames on **/tf**.

### 7.1.1 Applications of transformations between frames:

The following are some of the applications where the tf2 library can be used:

1.  Compute inverse and forward kinematics of multi-joint robots
2.  Carry out obstacle avoidance
3.  Convey location of robot or parts of a robot
4.  Convert sensor data from one reference to another
5.  Control robot about a particular point in space
6.  Analyze multiple robot data in world frame
7.  Reference external objects w.r.t robot frame

## 7.2 Demo using turtlesim

Enter the following command to install the necessary dependencies and compile the demo package.
`sudo apt-get install ros-$ROS_DISTRO-turtle-tf2 ros-$ROS_DISTRO-tf2-tools ros-$ROS_DISTRO-tf`

After compiling the *turtle_tf2* tutorial package, enter the following the run the demo:
`roslaunch turtle_tf2 turtle_tf2_demo.launch`

Once the turtlesim is started you can drive the center turtle around in the turtlesim using the keyboard arrow keys. On moving the center turtle, you will see that the other turtle also moves continuously to follow the turtle that you are driving around.

![](/Images/tf_1.png)

On entering the above command, you are essentially executing two main things:

1.  A TF2 broadcaster that is publishing the coordinate frames of both the turtles w.r.t the world frame.
2.  A TF2 listener that reads the transformations and uses it to calculate the direction in which turtle2 has to move to follow turtle1.

### 7.2.1 TF2 tools

#### view_frames:

**view_frames** is one of the TF2 tools that generates a diagram of the frames being broadcast by TF2 over ROS with the current TF2 tree. This diagram is stored as a pdf.

Syntax to run view_frames:
`rosrun tf2_tools view_frames.py`

This is what you will see:

```txt
Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...
```

To view the tree diagram, we just need to open the pdf. If evince is your default document viewer, then run the command `evince frames.pdf`

![](/Images/tf_vi_1.png)

On opening the pdf, we see a very simple tree diagram that depicts three frames - world frame, turtle1 frame, and turtle2 frame.

1.  The world frame is transformed to the turtle1 and to turtle2, seperately.
2.  We can see the Average rate, which is the publishing rate
3.  We can also see the most recent transform number, which should more or less coincide with the recording time (otherwise it is not publishing correctly)

![](/Images/tf_vi_2.png)

You can also listen to the TF being published, using the *echo*. There is a topic named **/tf** where ALL the TF are published. In simple systems like this one there is no problem, but as the system becomes more sophisticated, the quantity of data can be overwhelming. To tackle this, tf2 library provides you with tools that filters which tranformation you are interested in and just shows you that one.

![](/Images/tf_vi_3.png)

Running the following commands shows you the TF of the respective turtle only w.r.t the world frame.

![](/Images/tf_vi_4.png)

![](/Images/tf_vi_5.png)

#### tf_echo:

If you want to see the transform change as the two turtles move relative to each other, you can use **tf_echo**. tf_echo reports the transform between any two frames broadcast over ROS.

Syntax:
`rosrun tf tf_echo [reference_frame] [target_frame]`

On running `rosrun tf tf_echo turtle1 turtle2` you get the following information:

1.  Relative translation between the two turtles
2.  Relative rotation between the two turtles in terms of Quaternions, RPY (Roll Pitch Yaw)

![](/Images/echo_1.png)

![](/Images/echo_2.png)

#### tf_monitor:

### 7.2.2 Better visualization - rviz

rviz is a visualization tool that is useful for examining tf2 frames.

To open the turtle_tf2 file on rviz, enter the command: `rosrun rviz rviz` and open the relevant file. This is how your rviz environment will typically look like.

![](/Images/rviz_1.png)

![](/Images/rviz_2.png)

> Reference - https://youtu.be/Ra-nXIfPWdg

## 7.3 Quaternion and Roll-Pitch-Yaw Conversion

```python
#! /usr/bin/python2

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

roll = math.radians(30)
pitch = math.radians(50)
yaw = math.radians(75)

print('Roll: {}'.format(math.degrees(roll)))
print('Pitch: {}'.format(math.degrees(pitch)))
print('Yaw: {}'.format(math.degrees(yaw)))

quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
print("\nResulting quaternions:")

for i in range(0, 4):
    print(quaternion[i])


ori = tf.transformations.euler_from_quaternion(quaternion)
print('\nEuler from quaternion:')
for i in range(0, 3):
    print(math.degrees(ori[i]))	
```

**Execution:**

![](/Images/euler.png)

## 7.4 TF2 Publisher/Broadcaster

We shall first create a new catkin package called `learning_tf2` to better understand and demonstrate writing code for publishers and subscribers.

Syntax to create the new package:
`catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim`

![](/Images/n1.png)

Create a new directory called `nodes` inside the *learning_tf2* package. We will be storing all our python nodes here.

To create a broadcaster, make a new file called **tf2_broadcaster.py** under the `nodes` directory.

![](/Images/n2.png)

**Code:**

```python
#! /usr/bin/python2

# import rospy
import rospy
# import tf2 module
import tf2_ros
# import tf
import tf

import geometry_msgs.msg
import turtlesim.msg


def turtle_func(msg, turtlename):
    # TransformBroadcaster makes publishing of transforms easy. To use the TransformBroadcaster, we need to import the tf2_ros module.
    br = tf2_ros.TransformBroadcaster() 
    # We create a TransformStamped object which will contain the message to be published. 
    t = geometry_msgs.msg.TransformStamped()

    # Before stuffing the actual transform values we need to give the TransformStamped object the appropriate metadata.

    t.header.stamp = rospy.Time.now() # We need to give the transform being published a timestamp, which in our case will be the current time
    t.header.frame_id = "world" # name of parent frame
    t.child_frame_id = turtlename # name of child frame

    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    # convert angles from euler (radians/degrees) to quaternion 
    q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t) # publish the transform

if __name__=='__main__':
    rospy.init_node('tf2_broadcaster') # create node
    turtlename = rospy.get_param("~turtle")

    rospy.Subscriber('%s/pose' % turtlename,
                    turtlesim.msg.Pose,
                    turtle_func,
                    turtlename)

    rospy.spin()
```

![](/Images/br_1.png)

![](/Images/br_2.png)

![](/Images/br_3.png)

**Execution:**
To run the broadcaster, we first need to create a launch file. Inside your package, create a folder called `launch`, create a file called **start_demo.launch**.

![](/Images/br_4.png)

```html
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="sim" />
  <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen" />

  <node name="turtle1_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle1" />
  </node>
  <node name="turtle2_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle2" />
  </node>

</launch>
```

To execute, just run the command `roslaunch learning_tf2 start_demo.launch`.

![](/Images/br_5.png)

![](/Images/br_6.png)

![](/Images/br_7.png)

**Rviz:**

![](/Images/br_8.png)

## 7.5 TF2 Subscriber/Listener

```python
#! /usr/bin/python2

# import rospy
import rospy
# import tf2 module
import tf2_ros
# import math module
import math

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

# initialize listener node
rospy.init_node("tf2_listener")

# A listener object is created. Once the listener is created, it starts receiving tf2 transformations, and buffers them for up to 10 seconds.
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Spawn another turtle in the same turtlesim node (Refer to https://github.com/Bhaswanth-A/ROS-Theory/blob/main/Services.md#571-spawning-2-turtles-in-the-same-node )
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', Spawn)
turtle_name = rospy.get_param('turtle', 'turtle2')
spawner(4.0, 2.0, 0.0, turtle_name)

pub = rospy.Publisher('%s/cmd_vel' % turtle_name,
                      Twist, latch=True, queue_size=1)


r = rospy.Rate(10)

while not rospy.is_shutdown():

    try:
        # Gets the transformation from source frame to target frame (change turtle1 to carrot1 for frames example)
        trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

    # some math
    angular = 4 * math.atan2(trans.transform.translation.y,
                             trans.transform.translation.x)
    linear = 0.5 * math.sqrt(trans.transform.translation.x **
                             2 + trans.transform.translation.y ** 2)

    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    # publish the pose
    pub.publish(cmd)

    r.sleep()
```

![](/Images/l_1.png)

![](/Images/l_2.png)

![](/Images/l_3.png)

**Execution:**

Change the launch file **start_demo.launch** to this (only 1 line is added, nothing else is changed from before).

```html
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="sim" />
  <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen" />


  <node name="turtle1_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle1" />
  </node>
  <node name="turtle2_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle2" />
  </node>

  <node pkg="learning_tf2" type="tf2_listener.py" name="listener" output="screen" />

</launch>
```

To execute, just run the command `roslaunch learning_tf2 start_demo.launch`.

If you drive around turtle1 using your keyboard, you'll find the second turtle following the first one.

![](/Images/l_4.png)

![](/Images/l_5.png)

![](/Images/l_6.png)

**Rviz:**

![](/Images/l_7.png)

![](/Images/l_8.png)

## 7.6 Adding frames

Create a new file called **add_frame_1.py** under the `nodes` directory.

### 7.6.1 Example 1

#### Broadcaster 

```python
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped

def frames():

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "turtle1"
    t.child_frame_id = "carrot1"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = -2.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('frames')

    frames()
```

We create a new transform, from the parent "turtle1" to the new child "carrot1". The carrot1 frame is 2 meters offset from the turtle1 frame.

#### Listener

```python
#! /usr/bin/python2

import rospy
import tf2_ros
import math

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

rospy.init_node("tf2_listener")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', Spawn)
turtle_name = rospy.get_param('turtle', 'turtle2')
spawner(4.0, 2.0, 0.0, turtle_name)

pub = rospy.Publisher('%s/cmd_vel' % turtle_name,
                      Twist, latch=True, queue_size=1)


r = rospy.Rate(10)

while not rospy.is_shutdown():

    try:
		# parent frame changed to carrot1
        trans = tfBuffer.lookup_transform(turtle_name, 'carrot1', rospy.Time())

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

    angular = 4 * math.atan2(trans.transform.translation.y,
                             trans.transform.translation.x)
    linear = 0.5 * math.sqrt(trans.transform.translation.x **
                             2 + trans.transform.translation.y ** 2)

    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    pub.publish(cmd)

    r.sleep()
```

#### Launch file

```html
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="sim" />
  <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen" />


  <node name="turtle1_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle1" />
  </node>
  <node name="turtle2_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle2" />
  </node>

  <node pkg="learning_tf2" type="tf2_listener.py" name="listener" output="screen" />

  <node pkg="learning_tf2" type="add_frame_1.py" name="broadcaster_frames" output="screen" />

</launch>
```

![](/Images/fr_ex1_1.png)

![](/Images/fr_ex1_2.png)

### 7.6.2 Example 2

#### Broadcaster

```python
#! /usr/bin/python2

import rospy
import tf2_ros
import math

from geometry_msgs.msg import TransformStamped


def frames():

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "turtle1"
    t.child_frame_id = "carrot1"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        x = rospy.Time.now().to_sec() * math.pi

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 10 * math.sin(x)
        t.transform.translation.y = 10 * math.cos(x)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('frames')

    frames()
```

Instead of a fixed definition of our x and y offsets, we are using the sin and cos functions on the current time so that the offset of carrot1 is constantly changing.

#### Listener

```python
#! /usr/bin/python2

import rospy
import tf2_ros
import math

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

rospy.init_node("tf2_listener")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', Spawn)
turtle_name = rospy.get_param('turtle', 'turtle2')
spawner(4.0, 2.0, 0.0, turtle_name)

pub = rospy.Publisher('%s/cmd_vel' % turtle_name,
                      Twist, latch=True, queue_size=1)


r = rospy.Rate(10)

while not rospy.is_shutdown():

    try:
		# parent frame changed to carrot1
        trans = tfBuffer.lookup_transform(turtle_name, 'carrot1', rospy.Time())

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

    angular = 4 * math.atan2(trans.transform.translation.y,
                             trans.transform.translation.x)
    linear = 0.5 * math.sqrt(trans.transform.translation.x **
                             2 + trans.transform.translation.y ** 2)

    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    pub.publish(cmd)

    r.sleep()
```

#### Launch file

```html
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="sim" />
  <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen" />


  <node name="turtle1_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle1" />
  </node>
  <node name="turtle2_tf2_broadcaster" pkg="learning_tf2" type="tf2_broadcaster.py" respawn="false" output="screen">
    <param name="turtle" type="string" value="turtle2" />
  </node>

  <node pkg="learning_tf2" type="tf2_listener.py" name="listener" output="screen" />

  <node pkg="learning_tf2" type="add_frame_1.py" name="broadcaster_frames" output="screen" />

</launch>
```

![](/Images/fr_ex2_1.png)

![](/Images/fr_ex2_2.png)

## 7.7 Time travel

Let us see how we can get transformations from the past.

### 7.7.1 Example 1
Make the following changes in the listener file.

```python
while not rospy.is_shutdown():
    try:
        past = rospy.Time.now() - rospy.Duration(5)
        trans = tfBuffer.lookup_transform(turtle_name,'turtle1',past)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue

```
The above code asks for the pose of *turtle1* 5 seconds ago relative to turtle2 5 seconds ago. But this is not what we want. We want the pose of turtle1 5 seconds ago relative to the current pose of *turtle2*.

![](/Images/tt_1.png)

### 7.7.2 Example 2

```python
while not rospy.is_shutdown():
    try:
        past = rospy.Time.now() - rospy.Duration(5)
        trans = tfBuffer.lookup_transform_full(turtle_name, rospy.Time.now(), 'turtle1', past, 'world', rospy.Duration(1))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
```
The above code computes the pose of *turtle1* 5 seconds ago relative to the current pose of *turtle2*.

![](/Images/tt_2.png)
