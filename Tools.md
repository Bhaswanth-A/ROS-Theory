# ROS Tools

## rosbash

Documentation- http://wiki.ros.org/rosbash
rosbash is a package that contains some useful bash functions and adds tab-completion to a large number of the basic ROS utilities. When you source your setup file, you will implicitly get all bash-specific commands.

rosbash includes the following command-line utilities:

- roscd - change directory starting with package, stack, or location name
- rospd - pushd equivalent of roscd
- rosd - lists directories in the directory-stack
- rosls - list files of a ros package
- rosed - edit a file in a package
- roscp - copy a file from a package
- rosrun - run executables of a ros package

1.  rospd:
    rospd is the pushd equivalent of roscd, that allows you to navigate between different ros directories by keeping the multiple locations in a directory-stack, and allowing you to jump back to a ros directory that you were previously working on.
    
    ![](/Images/rospd.png)
      
2.  rosed:
    rosed allows you to edit files in a ROS package by typing the package name and the name of the file that you want to edit.
    
    ![](/Images/rosed.png)
    
3.  roscp:
    roscp allows you to copy a file from a ROS package by specifying the package name and the name of the file that you want to copy.
    
    ![](/Images/roscp.png)
    

## Common File system tools

1.  rospack:
    Documentation- https://docs.ros.org/en/independent/api/rospkg/html/rospack.html
    rospack is a command-line tool that is used to get information about ROS packages available on the filesystem. Below are listed some common rospack options-
    
    - `rospack find` \- returns the absolute path to the package
    - `rospack depends` \- returns a list of all the package’s dependencies
    - `rospack depends-on` \- returns a list of all the packages that depend on the given package
    - `rospack export` \- returns flags necessary for building and linking against a package
    - `rospack list` \- returns a list of all ROS packages on the filesystem
    
    ![](/Images/rospack.png)
    
2.  roscd:
    roscd allows us to change directory or subdirectory using a package name, stack name, or special location.
    
    ![](/Images/roscd.png)
    
    roscd log will take you to the ROS directory that contains log files. If no ROS program has been run yet, it will yield an error saying that it does not exist.
    
    ![](/Images/roscd_2.png)
    
3.  rosls:
    rosls allows you to view the contents of a package, stack or location.
    
    ![](/Images/rosls.png)
    
4.  rosparam:
    

## rosrun

rosrun is a ROS command-line utility that searches for a package for the requested program and runs it.
Syntax-
`rosrun <package> <executable>`

![](/Images/rosrun_1.png)

![](/Images/rosrun_2.png)

rosrun displays a sequence of timestamps, which in the above picture, prints the string ‘Hello World’ 10 times per second. This reduced frequency helps in recognizing any changes in the messages.
In UNIX, every program has a stream called standard output, or stdout. When a ‘Hello World’ program is run in a terminal, its stdout stream is sent and received by its parent terminal program which prints the text in a terminal emulator window.
So running a simple ‘Hello World’ program in ROS will have two nodes - one node that sends the stream of messages to other nodes, and the other node that sends messages to the terminal emulator program node.

## roslaunch

Although rosrun is great for starting single ROS nodes, most robot systems end up running tens or hundreds of nodes simultaneously. Since it would not be practical to call rosrun on each of these, ROS includes a tool for starting collections of nodes, called roslaunch.

roslaunch is a command line tool that helps run several nodes at a time, instead of using rosrun for each individual node.
Syntax-
`roslaunch PACKAGE LAUNCH_FILE`

roslaunch operates on launch files instead of nodes. Launch files are XML files that are a collection of nodes along with their topic remappings and other parameters. These files have the suffix *.launch*.

![](/Images/roslaunch_1.png)

roslaunch includes several other features such as the ability to launch programs on other computers in the network via ssh, to automatically respawn nodes that crash, etc.
roslaunch will also automatically run roscore, if it doesn’t already exist. However, pressing Ctrl+C will exit roscore as well, along with roslaunch.

### Creating a launch file

```html
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```
The launch tag `<launch>` is used to identify the file as a launch file.
Lines 3-9: starts two instances of the same node
Lines 11-14: start the mimic node with the topics input and output renamed to turtlesim1 and turtlesim2. This renaming will cause turtlesim2 to mimic turtlesim1.

![](/Images/roslaunch_2.png)

![](/Images/roslaunch_3.png)

## ROS Nodes

A node is a ROS program that performs a certain task. Nodes are combined together into a graph and communicate with each other using topics.

### Launching a node

Syntax-
`rosrun <package_name> <node_name> __name:=new_node_name __ns:=name_space topic:=new_topic`

### Controlling 2 turtles using different keyboards

![](/Images/1.png)

![](/Images/2.png)

**NOTE** \- We cannot have 2 turtlesims in the same namespace. Take a look at the below image to see what happens if we initialize 2 turtlebots in the same namespace.

![](/Images/3.png)

### Using rosnode

The rosnode command is used to display information about ROS nodes that are currently running.

![](/Images/4.png)

![](/Images/5.png)

rosout is the console log report mechanism in ROS. Documentation - http://wiki.ros.org/rosout