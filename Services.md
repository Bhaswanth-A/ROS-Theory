# 5. ROS Services

## 5.1 What are Services?

Services are synchronous calls, which when called by one node executes a function in another node. They are used only when a function/task needs to be executed occassionally, say when a robot needs to perform a very discrete task such as taking a high resolution picture using a camera, etc.

> Synchronous refers to an intereference with time, and means the that functions are performed one after the other. Messages, on the other hand, are asynchronous which means that they branch out into dfferent functions that execute simultaneously.

A **Server** provides a service by responding to a service call, and a **Client** requests for a service and accesses the service response.

> ## What is the difference between Services and Messges?
>
> ROS messages, which are transported using publishers and subscribers, are used whenever we need data to flow constantly and when we want to act on this data asynchronously.
ROS services, on the other hand, are used only when we require data at specific time or when we want a task to be executed only at particular instances. 
>
> To better understand this, take a look at the following example.
>
> 1. We have a robot that simulates it's environment in real-time. In such cases, we would use publishers/subscribers to send messages as that data flow needs to be constant. Also, we would want the robot to do other tasks as well, apart from just reading real-time data. If we use services, then the server/client would have to wait for a response/request and blocks the other code in the node, preventing other tasks from being executed,
>2. We have a robot that detects people in front of it. We would use services here as the node will wait for a person to come in front of it, then sends a request to the server and blocks the code while waiting for a response. Using messages here is pointless as we don't want to continuously check for people in front of the robot (it's just a one-time task).
>
>Reference: https://stackoverflow.com/questions/29458467/ros-service-and-message#:~:text=It%20is%20a%20similar%20example,the%20rest%20of%20its%20job.


## 5.2 Service files

Service files have an input and output call, and have the extension `.srv`. These files are present in the **srv** directory of a package. Here's how a typical service file is defined

```txt
int 64 a
int 64 b
---
int64 sum
```

![](/Images/serv_1.png)

In order for the service files to run, a few changes have to be made in the *package.xml* file and *CMakeLists.txt* file.

**In *package.xml***

```html
# The following lines need to be added at the end of the file
  <build_depend>message_generation</build_depend>
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
)
```

> `message_generation` was added in both the files and is done while creating messages as well. It works for both msg and srv.

```py
add_service_files(
  FILES
  Service1.srv
  Service2.srv
)
```

After the necessary changes are made, we need to run `catkin_make` or `catkin_make --only_pkg_with_deps <package_name>` to compile the workspace or a particular package.

Running `catkin_make` will create 2 additional classes for the service file as well. For example, if a service file is named add\_two\_int.srv, then the two classes add\_two\_intResponse and add\_two\_intRequest are created along with the previously existing add\_two\_int class. These classes allow us to interact with the service by calling for a *Response* or a *Request*.

## 5.3 Using rossrv

Documentation - http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

![](/Images/serv_2.png)

## 5.4 Using rosservice

![](/Images/serv_3.png)

> ### Difference between rossrv and rosservice
> 
> `rossrv` is a tool that provides us with information about all files ending in *.srv*
> `rosservice` is a tool that allows us to interact with *Servers* and *Clients* that are currently active.
> Reference - https://answers.ros.org/question/349148/rossrv-vs-rosservice/

## 5.5 Creating a simple Server

Before we get into creating a server, let us take a look at out service definition file.

![](/Images/server_1.png)

The same info can be gathered using the `rossrv` command, as shown below.

![](/Images/server_2.png)

Let us now create a server file, which would act as a python node for providing the service.
Here's the code for creating a simple server.

**Code:**

```python
#! /usr/bin/python2

import rospy
from beginner_tutorials.srv import add_two_int,add_two_intResponse

response = add_two_intResponse()

def add_ints(req):
    print('Adding {} + {} '.format(req.a,req.b))
    response.sum = req.a + req.b
    return response


rospy.init_node('add_two_int_server')

srv = rospy.Service('add_two_int',add_two_int,add_ints)

print('Server is ready')

rospy.spin()
```

### 5.5.1 Code Explanation

We import the `add_two_intResponse` class as we are writing code for a Server, which provides a response for the service that is called.

`response` is an object of the class `add_two_intResponse()`.

`srv = rospy.Service('add_two_int',add_two_int,add_ints)` This line creates an object named *srv* and declares/starts a service. The new service is given the name 'add\_two\_int' and has the service type `add_two_int`. **add_ints** is a callback function that performs a desired task.

The function **add_ints** takes a variable as a request and returns the sum of 'a' and 'b'.

**Execution:**

![](/Images/server_3.png)

## 5.6 Creating a simple Client

Like all python nodes, the client service file should also be created under the `src` directory of a package.

**Code:**

```python
#! /usr/bin/python2

import rospy
from beginner_tutorials.srv import add_two_int, add_two_intRequest


req = add_two_intRequest()
req.a = 4
req.b = 5

rospy.wait_for_service('add_two_int')

add_two_ints = rospy.ServiceProxy('add_two_int',add_two_int)
response = add_two_ints(req)
print(response.sum)
```

### 5.6.1 Code Explanation

The class *add\_two\_intRequest* is imported in a client node as we need to send a *Request* to the Server.

`req = add_two_intRequest()` : *req* is an object of the class *add\_two\_intRequest()*. The inputs for the object are given in the next two lines, which are to be sent to the server.

`rospy.wait_for_service('add_two_int')`: This is a method that blocks until the service named 'add\_two\_int' is available.

`add_two_ints = rospy.ServiceProxy('add_two_int',add_two_int)`: This is how we call a service of the service name *add\_two\_int* and service type *add\_two\_int*.

> **What is ServiceProxy?**
> 
> Before we get into that, what is a *Proxy* ?
> A proxy is a gateway between a client and a server (usually a website) that takes a request and performs a function. 
> 
> In a similar way, a ServiceProxy acts as an intermediary between a Client and a Server. It takes in the request from a Client and sends it to the Sever where it is passed into a function.

`response = add_two_ints(req)`: The request is passed into the service call line, and is sent to the server which returns the desired output.

`print(response.sum)`: The sum is then printed on the screen.

**NOTE - The server node needs to be running while a client node is being executed.**

**Execution:**

![](/Images/client_1.png)

![](/Images/client_2.png)

*Refer to the server code above to better understand how the service is being provided and why it returns the above outputs.*

## 5.7 A deeper dive into Services

Let us take a look at a few other interesting things that we can do using services.

### 5.7.1 Spawning 2 turtles in the same node

Here's a list of the services associated with turtlesim.
![](/Images/spawn_1.png)

The one which we are particularly interested in, to spawn 2 turtles in the same node, is the service named '/spawn'. Here's some info about the service '/spawn'.

![](/Images/spawn_2.png)

In order to spawn two turtles, we call a service and pass in the necessary parameters, which is demonstrated below

![](/Images/spawn_3.png)

### 5.7.2 Getting laser scan data using gazebo

Spawn your turtlebot3 on gazebo by running the following command
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

Place a block in front of your turtlebot3, as shown below

![](/Images/laser_1.png)

Let us take a look at the various topics associated with gazebo

![](/Images/laser_2.png)

Here, we find a topic named **/scan** which is pretty interesting. To learn more about this topic, let us do a `rostopic info`, as shown below.

![](/Images/laser_3.png)

Let us now take a look at what this message *sensors_msgs/LaserScan* has to offer.

![](/Images/laser_4.png)

Thus, this message provides us with several things related to the sensors output, and more importantly, gives us an array of all ranges of the laser scan. This is something that can be incredibly useful. However, subscribing to this would provide us with a list of 360 values (for 360 degrees), which is kinda unnecessary. 
Using services, we can have a synchronous callback function that provides us with data only for discrete inputs, such as giving us the range for a particular angle.

Let us now call for a service, by creating a server and a client. Before we do this, we would obviously need a service definition file present in the `srv` directory. 

Create a file named **gazebo_server.srv** under the *srv* directory, and have the following lines.
```txt
int64 direction
---
float32 distance
```
`int64 direction` would serve as an input while `float32 distance` serves as the output.

We need to make the necessary changes to *package.xml* file and *CMakeList.txt* file. This has been discussed above.

#### Creating a Server

##### Code
```python
#! /usr/bin/python2

import rospy
from sensor_msgs.msg import LaserScan
from beginner_tutorials.srv import gazebo_server, gazebo_serverResponse

rospy.init_node('My_Node', anonymous=True)

ls_obj = LaserScan()
response = gazebo_serverResponse()

def callback(val):
    global ls_obj
    ls_obj = val
    

def func_serv(request):
    global ls_obj
    response.distance = ls_obj.ranges[request.direction]
    return response.distance


sub = rospy.Subscriber('/scan', LaserScan, callback)

serv = rospy.Service('my_service', gazebo_server, func_serv)
rospy.spin()
```

##### Explanation

`ls_obj = LaserScan()`, `response = gazebo_serverResponse()` : Objects are created for the respective classes

`sub = rospy.Subscriber('/scan', LaserScan, callback)` : Subscribing to the topic **/scan** of the message type **LaserScan**, and the data from this is passed into the function named **callback** which stores the data in the object **ls_obj**.

`serv = rospy.Service('my_service', gazebo_server, func_serv)` : A service is started with the name **my_service**, service type **gazebo_server**, and a callback function named **func_serv** which returns the desired output based on the request.

##### Execution

![](/Images/laser_server.png)

#### Creating a Client

##### Code
```python
#! /usr/bin/python2

import rospy
from sensor_msgs.msg import LaserScan
from beginner_tutorials.srv import gazebo_server, gazebo_serverRequest

rospy.init_node('client_node')
rospy.wait_for_service('my_service')

req = gazebo_serverRequest()
req.direction = 1

srv = rospy.ServiceProxy('my_service', gazebo_server)

r = rospy.Rate(5)

while not rospy.is_shutdown():
    result = srv(req)
    print(result.distance)
    r.sleep()
```
##### Explanation

`rospy.wait_for_service('my_service')` : Waits for the service to start, which in our case is initialized by the server.

`req = gazebo_serverRequest()` : An object is created and the desired request is passed

`srv = rospy.ServiceProxy('my_service', gazebo_server)` : Calling for a service of the service name **my_service** and type **gazebo_server**

`result = srv(req)` : The request is passed into the proxy, which is sent to the server and executes the desired function.

`print(result.distance)` : The distance is printed on the screen.

**NOTE - The server node needs to be running while a client node is being executed.**

##### Execution

![](/Images/laser_client.png)
