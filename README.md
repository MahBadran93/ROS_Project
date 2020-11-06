<h1 align="center"> Robotics Project </h1> <br>
<h3 align="center"> Planning, Developing, Learning ROS </h3> <br>
<p align="center">
University Of Burgundy (VIBOT)
  <p align="center">
      <img src = "vibot.png" width=60>
  </p>
</p>

<h3 align="center">                       
Supervisors: <br>  
 Ralph SEULIN
</h3>
<h4 align="center">                       
Students: <br>  
 Mahmoud Badran,  Arsalan Khawaja
</h4>
<p align="center">
  <p align = "center">
     <img  src = "https://www.ros.org/news/2016/05/23/kinetic.png" width=400>
  </p>
</p>


<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Project Management (Technical Report)

- [Introduction](#introduction)
- [Project Tasks](#Project-Tasks)
- [Analysis OF Studied Techniques](#Analysis-OF-Studied-Techniques)
- [ROS Packages](#ROS-Packages)
- [Work Plan](#Work-Plan)
- [Conclusion](#Conclusion)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Introduction
Robotics Operating System (ROS), is a middleware, low level framework, to write robotic softwares. It can be cosidered as an API to make the process of developing a robotic related projects more flexible, and simplified. There will be no need for an extensive knowledge of the hardware in which it saves much effort and time in the development phase. It includes many libraries and tools which connects and control the robot manipulators, handle the comunication between multiple devices in a a working space. <br>
ROS is supported by many operating systems like ubunto, windows. Ubunto is the more stable operating system working with ROS. However, for the development of this project we are using the construct web plaform, which is an online robotics working environmant. The platform uses ubunto as the main operating system with ROS kinetic and uses the simulation of turtlebot 3, where we will be able to apply some of the techniques we have have learned to apply navigation and localization with map construction.  

## Project Tasks 
The project goal is to apply the learned **ROS** techniques and pakages to apply the navigation task on Turtlebot3:  
1- Create a script that moves the robot around with simple **/cmd_vel** publishing. See the range of
movement of this new robot model. <br>
2- Create the mapping launches, and map the whole environment. You have to finish with a clean map of
the full cafeteria. Setup the launch to be able to localize the Turtlebot3 robot. <br>
3- Set up the move base system so that you can publish a goal to move_base and Turtlebot3 can reach
that goal without colliding with obstacles. <br> 
4- Create a program that allows the Turtlebot3 to navigate within the environment following a set of
waypoints.

## Analysis OF Studied Techniques

Let's explain some important concepts that have been studied and will be important in developing the project : <br><br>
- **Nodes**: One of the most important concepts in ROS. We can describe nodes as a program (python, c++) to enable ROS to make communication tasks. A node can subscribe or publish to a topic, which will be explained. <br><br>
- **Topic**: The process of transmitting data between nodes. Some nodes are responsible for publishing some data to a specific topic where other nodes (subscribers) will be able to request these data ( messages ) from the topic.<br><br>
- **Messages** : Data structure which describes the data that ROS nodes publish or recieve. the nodes communicate, send messages, receive messages through topics.<br><br>
- **Services** : another way to transmit the data between nodes. it is a synchronous operation where the robot has to stop processing while waiting for a service response.<br><br>

<p align="center">
  <p align = "center">
     <img  src = "XCndTVvlwmaG.png" width=700>
  </p>
</p>

## ROS Packages



## Work Plan
#### Task 1: Robot Control <br><br>
Use **/cmd_vel** topic to move turtlebot3 around the environment. This topic is responsible for the **angular** and the **linear** velocity of the robot.<br>
we use **rostopic info /cmd_vel** to get information about the topic, after running the command we can see that this topic uses **Twist** type message. So, this topic recieves data of type Twist(angular and linear velocities ,(x,y,z)). <br> 
- Create a node that subscribe to **/scan** topic to get distance information from objects, walls. Also it publishes tarnslation and rotation data to      **/cmd_vel** topic to rotate and move the robot. <br>
<p align="center">
  <p align = "center">
     <img  src = "resources/r1.png" width=350>
    <img  src = "resources/r2.png" width=350>
    <img  src = "resources/r3.png" width=350>
    <img  src = "resources/r4.png" width=350>
  </p>
</p>
<br><br>

#### Task 2: Mapping & localization <br><br>
To start the autonomus navigation process, the robot must have a map of the environment to be able to recognize objects, walls where it will allow the robot to plann trajectories through environment. <br> 
In order to construct a map : <br> <br>

- We need to use **gmapping** package and run **slam_gmapping** node. 
    This node is implementing the gmapping **SLAM** algorithm. It creates a 2D map of the environment using the data the Robot is providing during movement like       laser data, in which it will be transformed to an Occumaoncy Grid Map (OGM) data format (**nav_msgs/OccupancyGrid.msg**) where it represents a 2-D grid map and each cell of the grid represents the occupancy ( if the cell is completely occupied or completely free). <br>
    Start the mapping process by executing this command: <br>
    <**rosrun gmapping slam_gmapping**> <br><br>
- In the mapping process, an important tool is used called **RViz**. It will help us in visulising th map creation process, it will allow us to see what the robot is covoring from the environment. <br>   
    <p align="center">
    <p align = "center">
       <img  src = "resources/screen.png" width=600>
    </p>
    </p>
- You can see in the figure above **Rviz**. In the left, we can see the displays which can be addded by us. we are interested in three displays which are:
    - **Map**: visulize the map. Topic is **/map** where it has message of type Occupancy Grid Map **OGM**, (**nav_msgs/OccupancyGrid.msg**)  <br> 
    - **LaserScreen**:  visualze what the Lazer on the robot is detecting. Topic is **/scan**<br>
    - **RobotModel**:  localize the Robot on the map.<br><br>
- After launnching **slam_gmapping** and **RViz**, we can start moving the robot by executing Kerbord control command:<br> 
  (**roslaunch turtlebot_teleop keyboard_teleop.launch**).<br> After moving the robot around all the places needed we should see the map fully occupied in **Rvis**<br>
    <p align="center">
    <p align = "center">
       <img  src = "resources/screen2.png" width=600>
    </p>
    </p> 
- The map can be saved using **map_server** package, it includes **map_savor** node  which will allow us to access the map data. 
    Execute this command : <br> 
    - **rosrun map_server map_savor -f <file_name>**  <br>
 After executing it will generate two files: <br><br>
    - **file_name.pgm:** PGM stands for Prtable Gray Map where it contains the Occupancy Grid Map(OGM) data. if we download the file and open it, it will look like this:  <br>
      <p align="center">
      <p align = "center">
         <img  src = "resources/mahmap.png" width=600>
      </p>
      </p> <br>
    Each cell ranges from 0 to 100 integer value where 0 means completely free and not occupied, 100 is completely occupied. <br><br>
    - **file_name.yaml:** This file contains the meta data of the generated map which contains these parametrs, image,resoulution, origin, occupied_thresh, free_thresh,negate. 
     


## Conclusion

## Sponsors [![Sponsors on Open Collective](https://opencollective.com/git-point/sponsors/badge.svg)](#sponsors)

Support this project by becoming a sponsor. Your logo will show up here with a link to your website. [[Become a sponsor](https://opencollective.com/git-point#sponsor)]

<a href="https://opencollective.com/git-point/sponsor/0/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/0/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/1/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/1/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/2/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/2/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/3/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/3/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/4/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/4/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/5/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/5/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/6/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/6/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/7/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/7/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/8/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/8/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/9/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/9/avatar.svg"></a>

## Acknowledgments

Thanks to [JetBrains](https://www.jetbrains.com) for supporting us with a [free Open Source License](https://www.jetbrains.com/buy/opensource).
