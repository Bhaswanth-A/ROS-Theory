# Introduction

## What is ROS?

ROS stands for Robot Operating System. Although the name implies that it is an OS, it is not. Rather, it is a framework that helps integrate the various parts of a robot by setting up communication between hardware and software of your robot, and between different processes going on within the software.
Let us consider the following example in order to understand it better. Suppose you are building a simple robot for a ‘fetch an item’ task, in which your robot needs to navigate a given environment, find the required item and bring it back to a specified location. The robot has various parts that perform different functions such as navigation, computer vision, etc. These different parts of the robot might be coded in different ways such as Python, OpenCV for computer vision, MATLAB etc. However, these parts cannot talk to each other, as they are written in different languages, due to which the robot will not know when it should stop/resume a certain task assigned to it. This is where ROS comes in. ROS helps integrate these various parts, thereby facilitating easy communication. The different parts now receive messages from other parts, thereby performing their functions more efficiently.

* * *

## ROS Graph

ROS Graph is a convenient mathematical way of representing a collection of programs and messages of a ROS system, typically as a graph. The various ROS programs, called nodes, communicate with each other by sending/receiving messages. The nodes in a ROS Graph are connected by "topics", which represents a stream of messages that nodes use to communicate with each other.

![05337bba41e6bf89cbd4b752c0b46caa.png](:/1f429d6148df4e34aadb4230ec4dcd52)

## roscore

roscore is a service that provides connection information to nodes so that they can find and transmit/receive messages with other nodes. Every node connects to roscore at startup to register details of the message streams it publishes and the streams to which it wishes to subscribe. When a new node appears, roscore provides it with the information that it needs to form a direct connection with other nodes publishing and subscribing to the same message topics.
<img src=":/b6d71ebd495c4b3099c1da803dedd808" alt="a6592e19005d82ecce6fa7269e84ef52.png" width="530" height="148" class="jop-noMdConv">

![3bbcd0784d2f51b9ccaa0862339d7e37.png](:/b5ddbc2b4c1b4560ab8fac837ed0adf3)

## catkin

catkin is the ROS build system, which is a set of tools that ROS uses to generate executable programs, scripts, libraries, etc. catkin comprises a set of CMake macros and custom Python scripts. Every catkin directory will contain two files - CMakeLists.txt and package.xml, that you need to make changes in order for things to work properly.

## Workspaces

A workspace is a set of files and directories that contain ROS code. You can have multiple workspaces but you can work on only one workspace at a time.

A catkin workspace is a directory where you build, modify, and install catkin packages.
Running catkin_make will create two new directories - devel and build, along with the previously existing src directory.

- Build is where catkin will store the results of some of its work such as libraries, etc.
- Devel contains a number of files and directories, including setup files. Running these setup files configures the system to use the workspace and run the necessary code. Hence it is necessary to source your workspace every time you open a new shell.

![ddf5b69c9b693f7255860945af50cc5d.png](:/6412c4861963464a8fb1087c58886a8f)

## ROS Packages

Documentation - http://wiki.ros.org/ROS/Tutorials/CreatingPackage
ROS software is organized into packages, which are built into the workspace and contained in the `src` directory.
The ROS ecosystem comprises thousands of publicly available packages in open repositories.

For a package to be considered a catkin package, it must meet the following requirements-

- Must contain package.xml file - provides meta information about the package
- Must contain CMakeLists.txt file - Describes how to build the code and where to install it
- Each package must have its own folder

![801563db0548084fb7d49c2b6c1825d2.png](:/630d13393da24054a487f777c270897f)

All packages must be created inside `~/catkin_ws/src`

### Creating a catkin package

To create a package the following syntax can be used:
`catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

![d201d7297fe1686fd1ea8dd6abba8b2b.png](:/b3bbe61df5194678a3dda999a1870aaa)

On creating a new package, the *CMakeLists.txt* file, *package.xml* file and the *src* directory come built-in automatically.

### Building the catkin workspace

Once a package has been created, it needs to be compiled in order for it to work.
-`catkin_make` \- Will compile the entire src directory and needs to be issued only in the `catkin_ws` directory.
-`catkin_make --only-pkg-with-deps <package_name>` \- Will only compile the selected package.

To add your workspace to the ROS environment, you need to source the generated setup file.

![d0dc604a3c6a26d902dc132369d684fe.png](:/0cea911c8ddd4358b8016dda8cdf64a8)

Once the package has been created, the Python nodes can be saved in the src directory of the package.

![caa3c644a5d0812a616cadf54e75e3cc.png](:/511dd5e3721f47a3965b4590c7c04129)

### Package dependencies

![22c65c3a990982b869760c7dcb309de7.png](:/cb89cc4632464036aa7a3425413d923b)

`rospack depends1 beginner_tutorials` returns a list of the first-order dependencies
`rospack depends beginner_tutorials` returns a list of all dependencies, direct and indirect.

These dependencies are stored in the package.xml file.

![7a2a4a4433ffbcee189f5681997f3337.png](:/df86f39ec2104711bb4664e394e6de70)

## package.xml

Documentation: http://wiki.ros.org/catkin/package.xml
The package.xml file provides meta information about the package such as package name, version number, authors, etc.

What does a package.xml file contain?

- The name of your package - You should not change this
- The version number
- A short description on the contents of the package
- Who’s responsible for maintaining the package and fixing the bugs?
- Licence details
- A URL, usually pointing to the ROS wiki
- Author tags
- Package dependencies

The following tags need to be nested within the &lt; package &gt; tag to make the package manifest complete-&lt; /package &gt;

- *&lt;name&gt;* \- The name of the package
- *&lt;version&gt;* \- The version number of the package (required to be 3 dot-separated integers)
- *&lt;description&gt;* \- A description of the package contents
- *&lt;maintainer&gt;* \- The name of the person(s) that is/are maintaining the package
- *&lt;license&gt;* \- The software license(s) under which the code is released.

![23be2c6c1c0295a0d397d3ad5b4577d7.png](:/91300c1cc1394d18a8b0d5550276cef4)

### Dependencies

These four types of dependencies are specified using the following respective tags:

- &lt;buildtool_depend&gt;: Build Tool Dependencies specify build system tools which this package needs to build itself. Typically the only build tool needed is catkin.
- &lt;build_depend&gt;: Build Dependencies specify which dependencies are needed to build this package.
- &lt;run_depend&gt;: Run Dependencies specify which dependencies are needed to run code in this package, or build libraries against this package.
- &lt;test_depend&gt;: Test Dependencies specify only additional dependencies for unit tests. They should never duplicate any dependencies already mentioned as build or run dependencies.

![238a6b2d8f881889237da85adce35651.png](:/b253a9aeaead4f50a561c67e3eb48aee)

### Metapackage

A metapackage is a group of multiple packages, with the following export tag in package.xml.

![68490d1933f60f6e4074d0e7f5ffc645.png](:/bca326a8030d49a885cf6aaeb50fdc1e)
