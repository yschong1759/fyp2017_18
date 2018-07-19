#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>

#include <fyp2017_18/cluster_info.h>
#include <fyp2017_18/table_info.h>
#include <fyp2017_18/drive_message.h>
#include <fyp2017_18/drive_feedback.h>

#define PI 3.141595654

// publisher
ros::Publisher drive_pose_pub;
ros::Publisher lifting_command_pub;

// declare global variables
fyp2017_18::cluster_info retrieve_cluster_data_odom;
fyp2017_18::table_info retrieve_table_data;
fyp2017_18::table_info retrieve_chair_data;
nav_msgs::Odometry current_pose;
fyp2017_18::drive_feedback drive_status;
std_msgs::Int8 platform_status;

float distance;
float backleg_gradient;
float state_1_end_x, state_1_end_y;
float state_2_end_x, state_2_end_y;

geometry_msgs::Point target_location_odom[4];
geometry_msgs::Point target_center_odom;
geometry_msgs::Point backleg_center;

int found_target = 0;       // 0 means no target found
int navigation_state = 0;   // there are 6 stages
                            // 1=adjust heading
                            // 2=align perpendicularly by driving only y direction
                            // 3=approach target driving x-direction only
                            // 4=deliver the target
                            // 5=leaving the target
                            // 6=ready for next

void get_odom(const nav_msgs::Odometry::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  current_pose = *msg;
}

void get_feedback(const fyp2017_18::drive_feedback::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
   drive_status = *msg;
}

void get_cluster_data(const fyp2017_18::cluster_info::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  retrieve_cluster_data_odom = *msg;
}

void get_lifting_mechanism_message(const std_msgs::Int8::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  platform_status = *msg;
}

void get_table_data(const fyp2017_18::table_info::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  if(found_target == 0) retrieve_table_data = *msg;
}

void get_chair_data(const fyp2017_18::table_info::ConstPtr& msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  if(found_target == 0) retrieve_chair_data = *msg;
}


void check_if_exist(int type) {
  /*
    iterate through the 4 legs of the detected table or chair
    identify the two legs that form correct entry side,
      for table, the long side is selected
      for chair, the side with chair back is selected
    check if the two legs are close to the result of clustering
      if yes, update the value using the results from clustering

    This function does not return. It updates to the global variable
    <backleg_gradient> and <target_location_odom>
  */
  int index = 0;
  int legs_seqeunce[4];    // to store the index of the <target_location_odom> that indicates the legs of entry side
  int found_entry_side_leg = 0;
  int table_entry_leg_count = 0;

  // find back leg
  for(int i=0; i<3; i++) {
    for(int j=i+1; j<4; j++) {
      distance = sqrt(pow(target_location_odom[j].x - target_location_odom[i].x, 2) + pow(target_location_odom[j].y - target_location_odom[i].y, 2));    // Pythagoras' theorem

      if(type == 0) {
        if(fabsf(distance - 0.29) < 0.025) {  // for chair
          found_entry_side_leg = 1;

          legs_seqeunce[index++] = i;
          legs_seqeunce[index++] = j;
          
          // calculate gradient
          backleg_gradient = -1 / ((target_location_odom[i].y - target_location_odom[j].y) / (target_location_odom[i].x - target_location_odom[j].x));
          if(backleg_gradient >= 1000 || backleg_gradient <= -1000) {
            backleg_gradient = PI / 2;    // radian
          } else {
            backleg_gradient = atan(backleg_gradient);    // radian
          }
          
          // calculate center of the entry side
          backleg_center.x = (target_location_odom[i].x + target_location_odom[j].x) / 2;
          backleg_center.y = (target_location_odom[i].y + target_location_odom[j].y) / 2;
          
          break;
        }
      } else if(type == 1) {
        if(fabsf(distance - 1.1) < 0.05) { // for table
          table_entry_leg_count++;
          
          backleg_gradient = -1 / ((target_location_odom[i].y - target_location_odom[j].y) / (target_location_odom[i].x - target_location_odom[j].x));
          if(backleg_gradient >= 1000 || backleg_gradient <= -1000) {
            backleg_gradient = PI / 2;    // radian
          } else {
            backleg_gradient = atan(backleg_gradient);    // radian
          }

          // calculate gradient
          // calculate center of the entry side
          if(table_entry_leg_count == 1) {
            backleg_center.x = (target_location_odom[i].x + target_location_odom[j].x) / 2;
            backleg_center.y = (target_location_odom[i].y + target_location_odom[j].y) / 2;
            legs_seqeunce[0] = i;
            legs_seqeunce[1] = j;
          }
          if(table_entry_leg_count == 2) {
            float temp_center_x = (target_location_odom[i].x + target_location_odom[j].x) / 2;
            float temp_center_y = (target_location_odom[i].y + target_location_odom[j].y) / 2;
            
            if(fabsf(backleg_center.x - temp_center_x) < 0.1) {
              if(backleg_center.y < temp_center_y) {
                backleg_center.x = temp_center_x;
                backleg_center.y = temp_center_y;
                legs_seqeunce[0] = i;
                legs_seqeunce[1] = j;
              }
            } else if(fabsf(backleg_center.y - temp_center_y) < 0.1) {
              if(backleg_center.x < temp_center_x) {
                backleg_center.x = temp_center_x;
                backleg_center.y = temp_center_y;
                legs_seqeunce[0] = i;
                legs_seqeunce[1] = j;
              }
            } else {
              float m = -1 / tan(backleg_gradient);
              float c = backleg_center.y - m * backleg_center.x;
              
              if((m * current_pose.pose.pose.position.x + c) * (m * temp_center_x + c) > 0) {
                backleg_center.x = temp_center_x;
                backleg_center.y = temp_center_y;
                legs_seqeunce[0] = i;
                legs_seqeunce[1] = j;
              }
            }
            found_entry_side_leg = 1;
          }

          break;
        }
      }
      if(found_entry_side_leg == 1) break;
    }
  }

  // check if the two legs are still in clustering result
  // if yes, update to target_location_odom
  for(int i=0; i<2; i++) {
    for(int j=0; j<retrieve_cluster_data_odom.count; j++) {
      if(fabsf(target_location_odom[legs_seqeunce[i]].x - retrieve_cluster_data_odom.points[j].x) < 0.1 && fabs(target_location_odom[legs_seqeunce[i]].y - retrieve_cluster_data_odom.points[j].y) < 0.1) {
        target_location_odom[legs_seqeunce[i]].x = retrieve_cluster_data_odom.points[j].x;
        target_location_odom[legs_seqeunce[i]].y = retrieve_cluster_data_odom.points[j].y;
        break;
      }
    }
  }
}

float compute_movement(int state, float stop) {
  /*
    Compute the desired location for
      navigation state 2 
      - the location is the intersection of 
        - the line perpendicular to the entry side passing through its center
        - the line parallel to the entry side passing through the current position of the robot
      navigation state 3
      - the location is the intersection of 
        - the line perpendicular to the entry side passing through its center
        - the line parallel to the entry side passing through the current position of the robot
    

    This function returns a float value. 
  */
  if(state == 0) {
    float m1, y_intercept_1;
    float m2, y_intercept_2;
    float x_coor, y_coor;
  
    m1 = tan(backleg_gradient);
    y_intercept_1 = backleg_center.y - m1 * backleg_center.x;
  
    m2 = -1 / m1;
    y_intercept_2 = state_1_end_y - m2 * state_1_end_x;
  
  
    y_coor = - (y_intercept_2 - y_intercept_1) / (pow(m1, 2) + 1) + y_intercept_2;
    x_coor = (y_intercept_2 - y_intercept_1) / (m1 - m2);

    return y_coor;
  }

  if(state == 1) {
    float distance = sqrt(pow(state_2_end_y - target_center_odom.y, 2) + pow(state_2_end_x - target_center_odom.x, 2));
    float stop_distance = stop;

    float ratio = (distance - stop_distance) / stop_distance;

    float stop_location_x = (ratio * target_center_odom.x + state_2_end_x) / (ratio + 1);
    float stop_location_y = (ratio * target_center_odom.y + state_2_end_y) / (ratio + 1);

    return stop_location_x;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "decision_making");
  ros::NodeHandle n;

  ros::Rate r(20); // 20 hz
  
  drive_pose_pub = n.advertise<fyp2017_18::drive_message>("pose_setpoint", 10);
  lifting_command_pub = n.advertise<std_msgs::Int8>("lifting_mechanism_message_input", 10);

  ros::Subscriber odom_sub = n.subscribe("odom", 10, get_odom);
  ros::Subscriber cluster_data_sub = n.subscribe("/cluster_data_odom", 10, get_cluster_data);
  ros::Subscriber table_data_sub = n.subscribe("/table_data_odom", 10, get_table_data);
  ros::Subscriber chair_data_sub = n.subscribe("/chair_data_odom", 10, get_chair_data);
  ros::Subscriber drive_feedback_sub = n.subscribe("drive_feedback", 10, get_feedback);
  ros::Subscriber lifting_mechanism_out_sub = n.subscribe("lifting_mechanism_message_output", 10, get_lifting_mechanism_message);


  // shifting sequence, the loop is to be completed in future
  // for simplicity, change order of 'c' and 't' in <seqeunce>
  // first element is the one targetted
  int sequence_count = 0;
  int seqeunce_total = 2;
  char seqeunce[2] = {'c', 't'};    // remember to check the setpoint in navigation state 3
  int index = 0;

  while (ros::ok()) {
    // based on the targetted, check if it is available
    if(found_target == 0) {
      if(seqeunce[sequence_count] == 't') {   // current target is table
        if(retrieve_table_data.count < 1) {
          printf("No tables\n");
        } else if(retrieve_table_data.count == 1) {
          // proceed to approach
          found_target = 1;
          navigation_state = 1;
          printf("Found a table, navigation_state: %i\n", navigation_state);

          for(int i=0; i<4; i++) {
            target_location_odom[i] = retrieve_table_data.legs[i];
          }
          target_center_odom = retrieve_table_data.center[0];
        } else {
          // wait for decision
          printf("number of tables: %i, pending decision\n", retrieve_table_data.count);
        }
      } else if(seqeunce[sequence_count] == 'c') {    // current target is table
        if(retrieve_chair_data.count < 1) {
          printf("No chairs\n");
        } else if(retrieve_chair_data.count == 1) {
          // proceed to approach
          found_target = 1;
          navigation_state = 1;
          printf("Found a chair, navigation_state: %i\n", navigation_state);

          for(int i=0; i<4; i++) {
            target_location_odom[i] = retrieve_chair_data.legs[i];
          }
          target_center_odom = retrieve_chair_data.center[0];
        } else {
          // wait for decision
          printf("number of chairs: %i, pending decision\n", retrieve_chair_data.count);
        }
      }
    }
    
    // if the target is found, proceed for approaching
    if(found_target == 1) {
      // check the identified entry side legs are available
      // check details from the comments in function <check_if_exist>
      if(seqeunce[sequence_count] == 'c') {
        check_if_exist(0);
      } else if(seqeunce[sequence_count] == 't') {
        check_if_exist(1);
      }
      printf("Check existence done\n");

      switch(navigation_state) {
        case 1: {
          // adjust heading
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_yaw_dir = 1;
          drive_settings.pose.theta = backleg_gradient;
          drive_settings.state = navigation_state;
          drive_pose_pub.publish(drive_settings);
          
          if(drive_status.status == 1) {
            navigation_state = 2;
            printf("drive_robot return %i, Going to state %i\n", drive_status.status, navigation_state);
            state_1_end_x = current_pose.pose.pose.position.x;
            state_1_end_y = current_pose.pose.pose.position.y;
          }
          break;
        }
        case 2: {
          // align perpendicularly by driving only y direction
          // drop the mechanism down when reach the target point
          // keep the plate
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_y_dir = 1;
          drive_settings.pose.y = compute_movement(0, 0);
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);
          
          if(drive_status.status == 2) {
            drive_settings.stop = 1;
            drive_settings.enable_y_dir = 0;
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            
            printf("drive_robot return %i\n", drive_status.status);
            
            std_msgs::Int8 command;

            if(platform_status.data == -1) {
              command.data = 0;
              lifting_command_pub.publish(command);
            } else {
              command.data = 5;    // dropping
              lifting_command_pub.publish(command);
            }

            printf("lifing mechanism is dropping\n");
            
            if(platform_status.data == 5) {
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();

              command.data = 3;    // keep plate
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();

              navigation_state = 3;
              printf("Going to state %i\n", navigation_state);

              drive_status.status = 0;
              platform_status.data = 0;
              state_2_end_x = current_pose.pose.pose.position.x;
              state_2_end_y = current_pose.pose.pose.position.y;
            }
          }
          break;
        }
        case 3: {
          // approach target driving x-direction only
          // stop at certain distance in front of table or chair
          // extend the plate (necessary for chair)
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          if(seqeunce[sequence_count] == 'c') {
            drive_settings.pose.x = compute_movement(1, 0.75);    // chair
          } else if(seqeunce[sequence_count] == 't') {
            drive_settings.pose.x = compute_movement(1, 0.3);    // table
          }
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);

          if(drive_status.status == 3) {
            drive_settings.stop = 1;
            drive_settings.enable_x_dir = 0;
            
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            
            printf("drive_robot return %i\n", drive_status.status);

            std_msgs::Int8 command;
            command.data = 4;    // extend plate
            lifting_command_pub.publish(command);
            printf("lifing mechanism is extending plate\n");

            if(platform_status.data == 4) {
              //command.data = 6;    // lifting table
              command.data = 7;    // lifting chair
              lifting_command_pub.publish(command);
              printf("lifing mechanism is lifting, wait for 15 seconds\n");
              ros::Duration(15.0).sleep();

              navigation_state = 4;
              printf("Going to state %i\n", navigation_state);

              drive_status.status = 0;
              platform_status.data = 0;
              
            }
          }
          break;
        }
        case 4: {
          // deliver the target
          // drive the robot to the location where the table or chair should be placed
          // when reached, drop the l-ifting mechanism
          // then, keep the plate
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          drive_settings.pose.x = target_center_odom.x + 0.5;    // move foward 0.5 meter away from center
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);

          if(drive_status.status == 4) {
            drive_settings.stop = 1;
            drive_settings.enable_y_dir = 0;
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);

            printf("drive_robot return %i\n", drive_status.status);

            std_msgs::Int8 command;
            command.data = 5;    // dropping
            lifting_command_pub.publish(command);
            printf("lifing mechanism is dropping\n");

            if(platform_status.data == 5) {
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();

              command.data = 3;    // keep plate
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              printf("lifing mechanism is keeping plate\n");

              navigation_state = 5;
              printf("Going to state %i\n", navigation_state);

              drive_status.status = 0;
              platform_status.data = 0;
            }

          }
          break;
        }
        case 5: {
          // leaving the target
          // set the lifting mechanism to keep pose
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          drive_settings.pose.x = target_center_odom.x - 1;    // move backwards to 1 meter away from center
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);

          if(drive_status.status == 5) {
            drive_settings.stop = 1;
            drive_settings.enable_x_dir = 0;
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);

            printf("drive_robot return %i\n", drive_status.status);
            
            std_msgs::Int8 command;
            command.data = 7;    // keeping
            lifting_command_pub.publish(command);
            printf("lifing mechanism is keeping\n");

            if(platform_status.data == 7) {
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();

              navigation_state = 6;
              printf("Going to state %i\n", navigation_state);
              
              drive_status.status = 0;
              platform_status.data = 0;
            }
          }
          break;
        }
        case 6: {
          // Ready for next
          printf("It is case 6\n");
          break;
        }
        default: {
          fyp2017_18::drive_message drive_settings;
          drive_settings.stop = 1;
          drive_pose_pub.publish(drive_settings);
          printf("navigation_state in default\n"); 
          break;
        }
      }
    } else {
      // no target, stop moving
      fyp2017_18::drive_message drive_settings;
      drive_settings.stop = 1;
      drive_pose_pub.publish(drive_settings);
      navigation_state = 0;
      printf("navigation_state: %i, stop driving...\n", navigation_state);
    }

    r.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
