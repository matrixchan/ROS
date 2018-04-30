# ROS Tutorials Notebook

## 1.1 Beginner Level

### 1. Installing and Configuring ROS Environment

* To check the environment variables of ros

  ```shell
  $ printenv | grep ROS
  ```

* Create a ROS Workspace

  create and build a catkin workspace:

  ```shell
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/
  $ catkin_make
  ```

  source the new setup.*sh file:

  ```shell
  $ source devel/setup.bash
  ```

  To echo the environment variables of ros:

  ```shell
  $ echo $ROS_PACKAGE_PATH
  /home/youruser/catkin_ws/src:/opt/ros/kinetic/share
  ```



### 2. Navigating the ROS Filesystem

```
rospack = ros + pack(age)
roscd = ros + cd
rosls = ros + ls
```

* roscd

  It allows you to change directory to a package or stack.

  ```
  $ roscd [locationname[/subdir]]
  ```

  ```
  $ roscd roscpp
  ```

  ```
  $ echo $ROS_PACKAGE_PATH
  ```

  ```
  $ roscd log
  ```

* rosls

  ls directly in a package by name .

  ```
  $ rosls [locationname[/subdir]]
  ```

  ```
  $ rosls roscpp_tutorials
  ```





### 3. Create a ROS Package

```shell
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

```
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src

#Now use the catkin_create_pkg script to create a new package called #'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

# Building a catkin workspace and sourcing the setup file
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
```

package dependencies

```
$ rospack depends1 beginner_tutorials 
```

```
$ roscd beginner_tutorials
$ cat package.xml
```





### 4. Building a ROS Package

* Using catkin_make

```
# In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

* Building zero to many catkin packages in a workspace follows this work flow:

```
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (optionally)
```



* Building Your Package

```
$ cd ~/catkin_ws/
$ ls src
$ catkin_make
```

