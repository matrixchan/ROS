# ROS Notebook

## Create  a ROS Workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

```
$ source devel/setup.bash
```

```
$ echo $ROS_PACKAGE_PATH
```

## [Create a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

* Create a catkin Package

  ```
  # You should have created this in the Creating a Workspace Tutorial
  $ cd ~/catkin_ws/src
  $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
  ```

* Building a catkin workspace

  ```
  $ cd ~/catkin_ws
  $ catkin_make
  ```

* Source the generated setup file

  ```
  $ . ~/catkin_ws/devel/setup.bash
  ```

* Package dependencies

  * First-order dependencies

  ```
  $ rospack depends1 beginner_tutorials 	
  ```

  * Indirect dependencies

  ```
  $ rospack depends1 rospy
  ```
  

## [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)

* roscore

  `roscore` is the first thing you should run when using ROS.

  ```
  $ roscore
  ```

* Using rosnode

  ```
  $ rosnode list
  ```

  ```
  $ rosnode info /rosout
  ```

* Using rosrun

  * Usage:

    ```
    $ rosrun [package_name] [node_name]
    ```

  * run the turtlesim_node in the turtlesim package:

    ```
    $ rosrun turtlesim turtlesim_node
    ```

## [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

* Setup

  * run roscore

    ```
    $ roscore
    ```

  * run turtlesim node

    ```
    $ rosrun turtlesim turtlesim_node
    ```

  * turtle keyboard teleoperation

    ```
    $ rosrun turtlesim turtle_teleop_key
    ```

* ROS Topics

  * Using rqt_graph

    ```
    $ rosrun rqt_graph rqt_graph	
    ```

  * rostopic

    ```
    $ rostopic -h
    ```

    ```
    $ rostopic 
    ```

    `rostopic echo` shows the data published on a topic. 

    Usage:

    ```
    rostopic echo [topic]
    ```

    ```
    $ rostopic echo /turtle1/cmd_vel
    $ rostopic echo /turtle1/command_velocity
    ```

  * Using rostopic list

    `rostopic list` returns a list of all topics currently subscribed to and published.

    ```
    $ rostopic list -h
    ```

    ```
    $ rostopic list -v
    ```

  * Using rostopic pub

    `rostopic pub` publishes data on to a topic currently advertised. 

    Usage:

    ```
    rostopic pub [topic] [msg_type] [args]
    ```

    ```
    $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    ```

  * Using rostopic hz

    ```
    $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
    ```

    Let's see how fast the `turtlesim_node` is publishing `/turtle1/pose`: 

    ```
    $ rostopic hz /turtle1/pose
    ```

  * Using rqt_plot

    `rqt_plot` displays a scrolling time plot of the data published on topics.

    