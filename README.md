fyp2017_18
----------

This is a ROS source code for a Final Year Project in NTU MAE, under supervision of Prof Gerald Seet. 

This package provides a solution to the robot to identify a 4-legged table or chair using a scanning laser rangefinder. Then, the robot is navigated to approach, lift and transport the target. 

This README hopes to give a great head start to the next person or team, to familiar with the current progress of development and hence extending the robot capabilities. 

Operation
---------

### On real robot

```
start odom              : rosrun odom odom_node
start hokuyo            : roslaunch fyp2017_18 laser.launch
start arduino           : roslaunch fyp2017_18 arduino_launch.launch
rviz                    : rosrun rviz rviz
table_detect_revised    : rosrun fyp2017_18 table_detect_revised
chair_detect_revised    : rosrun fyp2017_18 chair_detect_revised
to_odom                 : rosrun fyp2017_18 to_odom
odom_to_rpy             : rosrun fyp2017_18 odom_to_rpy
decision_making         : rosrun fyp2017_18 decision_making
drive_robot             : rosrun fyp2017_18 drive_robot
```

### On simulator

```
run Gazebo with world loaded        : rosrun mybot_gazebo mybot_world.launch
Connect Arduino <optional>          : rosrun fyp2017_18 arduino_launch.launch
rviz                                : rosrun mybot_description mybot_description.launc
table_detect_revised                : rosrun fyp2017_18 table_detect_revised
chair_detect_revised                : rosrun fyp2017_18 chair_detect_revised
to_odom                             : rosrun fyp2017_18 to_odom
odom_to_rpy                         : rosrun fyp2017_18 odom_to_rpy
decision_making                     : rosrun fyp2017_18 decision_making
drive_robot                         : rosrun fyp2017_18 drive_robot

```

Lifiting Mechanism Arduino Code
-------------------------------

`lifting_mechanism/lifting_mechanism.ino` is in used. It has been configured to communicate with ROS with several topics. 

For development, you may use `lifting_mechanism_serial_command/lifting_mechanism_serial_command.ino`, to interact with the board via Serial Monitor.

For debugging, you may use `recover/recover.ino`.

You may use `test_blink/test_blink.ino` as a testbed, to understand how Arduino and ROS interact.


Note
----

* files not in used in `/src`:
    - clustering.cpp, table_detection.cpp, chair_detection.cpp
        
        In the current workflow, `table_detect_revised` and `chair_detect_revised` is performing clustering to process the data from scanning laser rangefinders, and subseqeuntly execute the algorithm searching table and chair respectively. 

        ```
            clustering -------- table detection algorithm
                        
            clustering -------- chair detection algorithm    
        ```

        Since the clustering process is common in the searching a table or chair, it was proposed to optimize the process. The cluster data from clustering process is passed to the table and chair detection algorithm respectively.

        ```
                            |--- table detection algorithm
                            |
            clustering -----|    
                            |
                            |--- chair detection algorithm
        ```

        However, the latter is not able to work properly. Further investigation is needed.

    - chair_detect.cpp, table_detect.cpp
        
        This is the previous code. It performs clustering as well, then it takes any 4 from the identified clusters, compute the distances between the points, and compare them with the template distance. It may not be efficient when the number of clusters increases. 

        The files are kept for future reference. 

* Remember to check value of constant `ROBOT`

Troubleshooting
---------------

1. Unable to marker visualization in Rviz
    
    Check the header id of the corresponding marker.

2. No data from laser scanning rangefinder
    
    Check if the topic of the laser scanner is subscribed properly