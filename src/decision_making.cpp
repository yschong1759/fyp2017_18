#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <fyp2017_18/entry_point.h>
#include <fyp2017_18/cluster_info.h>
#include <fyp2017_18/table_info.h>
#include <fyp2017_18/drive_message.h>
#include <fyp2017_18/drive_feedback.h>

#define PI 3.141595654

ros::Publisher drive_pose_pub;
ros::Publisher lifting_command_pub;

fyp2017_18::entry_point retrieve_table_entry_points;
fyp2017_18::entry_point retrieve_chair_entry_points;
fyp2017_18::table_info retrieve_table_data;
fyp2017_18::table_info retrieve_chair_info;
fyp2017_18::cluster_info retrieve_backleg_info;
fyp2017_18::table_info retrieve_table_info;
fyp2017_18::table_info retrieve_chair_data;
fyp2017_18::cluster_info retrieve_cluster_data_odom;
nav_msgs::Odometry current_pose;
geometry_msgs::Pose2D desired_pose;
fyp2017_18::drive_feedback status;
geometry_msgs::Point backleg;
std_msgs::Int8 platform_status;

float distance;
float gradient, y_intercept;
float yaw_gradient;
float table_side_length = 1.1;

float backleg_gradient;
float state_1_end_x, state_1_end_y;
float state_2_end_x, state_2_end_y;

geometry_msgs::Point target_location_hokuyo[4];
geometry_msgs::Point target_location_odom[4];
geometry_msgs::Point target_center_odom;



int found_target = 0;    // 0 means no target found
int navigation_state = 0;    // 1=adjust yaw, 2=drive y, 3=drive x
geometry_msgs::Point backleg_center;

void compute_table_orientation(fyp2017_18::table_info table_group, int index) {
  y_intercept = 0;
  for(int i=0; i<(table_group.legs_number-1); i++) {
    for(int j=i+1; j<table_group.legs_number; j++) {
      distance = sqrt(pow(table_group.legs[4 * index + j].x - table_group.legs[4 * index + i].x, 2) + pow(table_group.legs[4 * index + j].y - table_group.legs[4 * index + i].y, 2));    // Pythagoras' theorem
      if(fabsf(distance - table_side_length) < 0.1) {
        printf("distance: %f\n", distance);
        gradient = 1 / ((table_group.legs[4 * index + j].y - table_group.legs[4 * index + i].y) / (table_group.legs[4 * index + j].x - table_group.legs[4 * index + i].x));
        yaw_gradient = atan2(table_group.center[index].y, table_group.center[index].x);
        float temp = atan(gradient);
        if(abs(temp) < 0.1) {
          y_intercept = table_group.center[index].y;
          break;
        } else if(temp > 89 || temp < -89) {
          y_intercept = 0;
        } else {
          y_intercept = table_group.legs[4 * index + j].y - gradient * table_group.legs[4 * index + j].x;
          break;
        }
      }
    }
    if(y_intercept != 0.0) {
      printf("gradient: %f, y_intercept: %f\n", gradient, y_intercept);
      break;
    }
  }
}

// float compute_side_movement() {
//   float perpendicular_gradient = gradient;
//   float y_coor;
//   if(atan(perpendicular_gradient) > 89 || atan(perpendicular_gradient) < -89) {
//     printf("y_coor: %f\n", y_intercept);
//     return y_intercept;
//   } else {
//     y_coor = ((perpendicular_gradient) * (current_pose.x + (perpendicular_gradient) * current_pose.y) + y_intercept) / (pow((perpendicular_gradient), 2) + 1);
//   }
//   printf("y_coor: %f\n", y_coor);
//   return y_coor;
// }

// float compute_forward_movement(float safety_distance) {
//   if(current_pose.x > safety_distance) {
//     return safety_distance - 0.8;
//   } else {
//     return safety_distance + 0.8;
//   }
// }

void get_tables_info(const fyp2017_18::table_info::ConstPtr& tables_msg) {
  double begin = ros::Time::now().toSec();
  retrieve_table_info = *tables_msg;
  double end = ros::Time::now().toSec();

  printf("get_tables_info used time: %f\n", end - begin);
}

void get_chairs_info(const fyp2017_18::table_info::ConstPtr& chairs_msg) {
  double begin = ros::Time::now().toSec();  
  retrieve_chair_info = *chairs_msg;
  double end = ros::Time::now().toSec();

  printf("get_chairs_info used time: %f\n", end - begin);  
}

void get_odom(const nav_msgs::Odometry::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();
  current_pose = *msg;
  double end = ros::Time::now().toSec();

  printf("get_odom used time: %f\n", end - begin);  
}

void get_feedback(const fyp2017_18::drive_feedback::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();   
   status = *msg;

  double end = ros::Time::now().toSec();
  printf("get_feedback used time: %f\n", end - begin);   
}

void get_cluster_data(const fyp2017_18::cluster_info::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();
  // tf::TransformListener listener_global(ros::Duration(1));

  // geometry_msgs::PointStamped before_to_odom[msg->count];
  // geometry_msgs::PointStamped after_to_odom[msg->count];
  // geometry_msgs::Point temp;

  // for(int i=0; i<msg->count; i++) {
  //   before_to_odom[i].header.frame_id = "hokuyo";
  //   before_to_odom[i].header.stamp = ros::Time();

  //   before_to_odom[i].point.x = msg->points[i].x;
  //   before_to_odom[i].point.y = msg->points[i].y;
  //   before_to_odom[i].point.z = msg->points[i].z;
  // }

  // try {
  //   for(int i=0; i<msg->count; i++) {
  //     listener_global.waitForTransform("/hokuyo", "/odom",
  //                             ros::Time::now(), ros::Duration(0.5));
  //     listener_global.transformPoint("odom", before_to_odom[i], after_to_odom[i]);
  //   }
  // } catch(tf::TransformException& ex) {
  //     ROS_ERROR("Recevied exception, %s", ex.what());
  // }
  
  // for(int i=0; i<msg->count; i++) {
  //   temp.x = after_to_odom[i].point.x;
  //   temp.y = after_to_odom[i].point.y;
  //   temp.z = after_to_odom[i].point.z;
  //   //printf("transformed coordinate: %f %f %f\n", after_to_odom[i].point.x, after_to_odom[i].point.y, after_to_odom[i].point.z);
  //   retrieve_cluster_data_odom.points.push_back(temp);
  // }

  retrieve_cluster_data_odom = *msg;

  double end = ros::Time::now().toSec();
  printf("get_cluster_data used time: %f\n", end - begin);  
}

void get_lifting_mechanism_message(const std_msgs::Int8::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();  
  platform_status = *msg;

  double end = ros::Time::now().toSec();
  printf("get_lifting_mechanism_message used time: %f\n", end - begin);  
}

// void get_table_entry_points(const fyp2017_18::entry_point::ConstPtr& msg) {
//   retrieve_table_entry_points = *msg;
// }

// void get_chair_entry_points(const fyp2017_18::entry_point::ConstPtr& msg) {
//   retrieve_chair_entry_points = *msg;
// }

void use_backleg_info() {
  int count = retrieve_backleg_info.count;
  printf("backleg count: %i\n", count);
  geometry_msgs::Point possible_points[20];
  float distance;
  //float backleg_length = 0.28;
  float backleg_length = 1.1;

  int index = 0;

  for(int i=0; i<count; i++) {
    if(fabsf(retrieve_backleg_info.points[i].y) < 1) {
      possible_points[index] = retrieve_backleg_info.points[i];
      index++;
      printf("backleg index: %i\n", index);
    }
  }

  for(int i=0; i<(index-1); i++) {
    for(int j=i; j<index; j++) {
      distance =  sqrt(pow(possible_points[j].x - possible_points[i].x, 2) + pow(possible_points[j].y - possible_points[i].y, 2));    // Pythagoras' theorem
      if(fabsf(distance - backleg_length) < 0.05) {
        backleg_center.x = (possible_points[j].x + possible_points[i].x) / 2;
        backleg_center.y = (possible_points[j].y + possible_points[i].y) / 2;
      }
    }
  }
}

void get_table_data(const fyp2017_18::table_info::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();  
  if(found_target == 0) retrieve_table_data = *msg;
  double end = ros::Time::now().toSec();
  
  printf("get_table_data used time: %f\n", end - begin); 

  

}

void get_chair_data(const fyp2017_18::table_info::ConstPtr& msg) {
  double begin = ros::Time::now().toSec();  
  double end = ros::Time::now().toSec();
  if(found_target == 0) retrieve_chair_data = *msg;
  printf("get_chair_data used time: %f\n", end - begin); 
}

void transfer_to_odom(geometry_msgs::Point * original_frame, int count, geometry_msgs::Point * results) {
  tf::TransformListener listener_global(ros::Duration(1));

  geometry_msgs::PointStamped before_to_odom[count];
  geometry_msgs::PointStamped after_to_odom[count];

  for(int i=0; i<count; i++) {
    before_to_odom[i].header.frame_id = "hokuyo";
    before_to_odom[i].header.stamp = ros::Time();

    before_to_odom[i].point.x = original_frame[i].x;
    before_to_odom[i].point.y = original_frame[i].y;
    before_to_odom[i].point.z = original_frame[i].z;
  }

  try {
    for(int i=0; i<count; i++) {
      listener_global.waitForTransform("/hokuyo", "/odom",
                              ros::Time::now(), ros::Duration(0.5));
      listener_global.transformPoint("odom", before_to_odom[i], after_to_odom[i]);
    }
  } catch(tf::TransformException& ex) {
      ROS_ERROR("Recevied exception, %s", ex.what());
  }

  for(int i=0; i<count; i++) {
    results[i].x = after_to_odom[i].point.x;
    results[i].y = after_to_odom[i].point.y;
    results[i].z = after_to_odom[i].point.z;
    //printf("transformed coordinate legs: %f %f %f\n", after_to_odom[i].point.x, after_to_odom[i].point.y, after_to_odom[i].point.z);
  }

}

void check_if_exist(int type) {
  int index = 0;
  int legs_seqeunce[4];
  int found_short_leg = 0;
  int table_short_leg_count = 0;

  // find back leg
  for(int i=0; i<3; i++) {
    for(int j=i+1; j<4; j++) {
      distance = sqrt(pow(target_location_odom[j].x - target_location_odom[i].x, 2) + pow(target_location_odom[j].y - target_location_odom[i].y, 2));    // Pythagoras' theorem
      printf("Checking distance: %f\n", distance);
      if(type == 0) {
        if(fabsf(distance - 0.29) < 0.025) {  // for chair
          printf("Found short legs\n");
          printf("leg 1: %f %f\n", target_location_odom[i].x, target_location_odom[i].y);
          printf("leg 2: %f %f\n", target_location_odom[j].x, target_location_odom[j].y);
          legs_seqeunce[index++] = i;
          legs_seqeunce[index++] = j;
          found_short_leg = 1;
          backleg_gradient = -1 / ((target_location_odom[i].y - target_location_odom[j].y) / (target_location_odom[i].x - target_location_odom[j].x));
          if(backleg_gradient >= 1000 || backleg_gradient <= -1000) {
            backleg_gradient = PI / 2;    // radian
          } else {
            backleg_gradient = atan(backleg_gradient);    // radian
          }
          printf("table backleg_gradient: %f\n", backleg_gradient);
          backleg_center.x = (target_location_odom[i].x + target_location_odom[j].x) / 2;
          backleg_center.y = (target_location_odom[i].y + target_location_odom[j].y) / 2;
          printf("backleg x: %f, y: %f\n", backleg_center.x, backleg_center.y);
          break;
        }
      } else if(type == 1) {
        if(fabsf(distance - 1.1) < 0.05) { // for table
          printf("Found long legs\n");
          printf("leg 1: %f %f\n", target_location_odom[i].x, target_location_odom[i].y);
          printf("leg 2: %f %f\n", target_location_odom[j].x, target_location_odom[j].y);

          table_short_leg_count++;
          backleg_gradient = -1 / ((target_location_odom[i].y - target_location_odom[j].y) / (target_location_odom[i].x - target_location_odom[j].x));
          if(backleg_gradient >= 1000 || backleg_gradient <= -1000) {
            backleg_gradient = PI / 2;    // radian
          } else {
            backleg_gradient = atan(backleg_gradient);    // radian
          }
          printf("backleg_gradient: %f\n", backleg_gradient);
          if(table_short_leg_count == 1) {
            backleg_center.x = (target_location_odom[i].x + target_location_odom[j].x) / 2;
            backleg_center.y = (target_location_odom[i].y + target_location_odom[j].y) / 2;
            legs_seqeunce[0] = i;
            legs_seqeunce[1] = j;
          }
          if(table_short_leg_count == 2) {
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
              //printf("m: %f, c: %f", m, c);
              if((m * current_pose.pose.pose.position.x + c) * (m * temp_center_x + c) > 0) {
                backleg_center.x = temp_center_x;
                backleg_center.y = temp_center_y;
                legs_seqeunce[0] = i;
                legs_seqeunce[1] = j;
              }
            }
            printf("backleg x: %f, y: %f\n", backleg_center.x, backleg_center.y);
            found_short_leg = 1;
          }
          break;
      }
    }
    if(found_short_leg == 1) break;
  }
}

  for(int i=0; i<2; i++) {
    for(int j=0; j<retrieve_cluster_data_odom.count; j++) {
      if(fabsf(target_location_odom[legs_seqeunce[i]].x - retrieve_cluster_data_odom.points[j].x) < 0.1 && fabs(target_location_odom[legs_seqeunce[i]].y - retrieve_cluster_data_odom.points[j].y) < 0.1) {
        printf("Still there...\n");
        printf("from cluster data: %f %f\n", retrieve_cluster_data_odom.points[j].x, retrieve_cluster_data_odom.points[j].y);
        printf("from table data: %f %f\n", target_location_odom[legs_seqeunce[i]].x, target_location_odom[legs_seqeunce[i]].y);
        target_location_odom[legs_seqeunce[i]].x = retrieve_cluster_data_odom.points[j].x;
        target_location_odom[legs_seqeunce[i]].y = retrieve_cluster_data_odom.points[j].y;
        break;
      }
    }
  }
}

float compute_movement(int state, float stop) {
  if(state == 0) {

    float m1, y_intercept_1;
    float m2, y_intercept_2;
    float x_coor, y_coor;
  
    m1 = tan(backleg_gradient);
    y_intercept_1 = backleg_center.y - m1 * backleg_center.x;
    printf("y_intercept_1: %f\n", y_intercept_1);
  
    m2 = -1 / m1;
    y_intercept_2 = state_1_end_y - m2 * state_1_end_x;
  
    printf("y_intercept_2: %f\n", y_intercept_2);
  
    y_coor = - (y_intercept_2 - y_intercept_1) / (pow(m1, 2) + 1) + y_intercept_2;
    x_coor = (y_intercept_2 - y_intercept_1) / (m1 - m2);
    printf("y_coor: %f\n", y_coor);
    // float distance;
    // float temp_center_x, temp_center_y;
    // // return y_coor;
    // for(int i=0; i<3; i++) {
    //   for(int j=i+1; j<4; j++) {
    //     distance = sqrt(pow(retrieve_table_info.legs[j].x - retrieve_table_info.legs[i].x, 2) + pow(retrieve_table_info.legs[j].y - retrieve_table_info.legs[i].y, 2));    // Pythagoras' theorem
    //     if(fabsf(distance - 1.1) < 0.1) {
    //       temp_center_x = (retrieve_table_info.legs[i].x + retrieve_table_info.legs[j].x) / 2;
    //       temp_center_y = (retrieve_table_info.legs[i].y + retrieve_table_info.legs[j].y) / 2;
    //       printf("temp_center_x: %f\n", temp_center_x);
    //     }
    //   }
    // }

    return y_coor;
  }

  if(state == 1) {
    float distance = sqrt(pow(state_2_end_y - target_center_odom.y, 2) + pow(state_2_end_x - target_center_odom.x, 2));
    printf("distance = %f\n", distance);
    float stop_distance = stop;

    float ratio = (distance - stop_distance) / stop_distance;

    float stop_location_x = (ratio * target_center_odom.x + state_2_end_x) / (ratio + 1);
    float stop_location_y = (ratio * target_center_odom.y + state_2_end_y) / (ratio + 1);

    printf("stop location: %f %f\n", stop_location_x, stop_location_y);

    return stop_location_x;

    // temporarily skip above
    //return target_center_odom.x;

  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "decision_making");
  ros::NodeHandle n;

  ros::Rate r(20); // 10 hz
  
  drive_pose_pub = n.advertise<fyp2017_18::drive_message>("pose_setpoint", 10);
  lifting_command_pub = n.advertise<std_msgs::Int8>("lifting_mechanism_message_input", 10);

  //ros::Subscriber tables_sub = n.subscribe("table_data", 10, get_tables_info);
  //ros::Subscriber chairs_sub = n.subscribe("chair_data", 10, get_chairs_info);

  ros::Subscriber odom_sub = n.subscribe("odom", 10, get_odom);
  //ros::Subscriber cluster_sub = n.subscribe("visualize_cluster", 10, get_cluster);
  //ros::Subscriber table_entry_point_sub = n.subscribe("/visualize_table_entry_points", 10, get_table_entry_points);
  //ros::Subscriber chair_entry_point_sub = n.subscribe("/visualize_chair_entry_points", 10, get_chair_entry_points);
  ros::Subscriber cluster_data_sub = n.subscribe("/cluster_data_odom", 10, get_cluster_data);
  ros::Subscriber table_data_sub = n.subscribe("/table_data_odom", 10, get_table_data);
  ros::Subscriber chair_data_sub = n.subscribe("/chair_data_odom", 10, get_chair_data);
  ros::Subscriber drive_feedback_sub = n.subscribe("drive_feedback", 10, get_feedback);
  ros::Subscriber lifting_mechanism_out_sub = n.subscribe("lifting_mechanism_message_output", 10, get_lifting_mechanism_message);

  int sequence_count = 0;
  int seqeunce_total = 2;
  char seqeunce[2] = {'c', 't'};
  int index = 0;

  while (ros::ok()) {
    //fyp2017_18::table_info tables = retrieve_table_info;
    double begin = ros::Time::now().toSec();  
    if(found_target == 0) {
      if(seqeunce[sequence_count] == 't') {
        if(retrieve_table_data.count < 1) {
          printf("No tables %i\n", retrieve_table_data.count);
        } else if(retrieve_table_data.count == 1) {
          // proceed to approach
          found_target = 1;
          navigation_state = 1;
          printf("Found a table, navigation_state: %i\n", navigation_state);

          for(int i=0; i<4; i++) {
            target_location_odom[i] = retrieve_table_data.legs[i];
            printf("target_location_odom, x: %f y: %f\n", target_location_odom[i].x, target_location_odom[i].y);
          }
          target_center_odom = retrieve_table_data.center[0];
          //transfer_to_odom(target_location_hokuyo, 4, target_location_odom);
        } else {
          // wait for decision
          printf("count: %i\n", retrieve_table_data.count);
        }
      } else if(seqeunce[sequence_count] == 'c') {
        if(retrieve_chair_data.count < 1) {
          printf("No chairs %i\n", retrieve_chair_data.count);
        } else if(retrieve_chair_data.count == 1) {
          // proceed to approach
          found_target = 1;
          navigation_state = 1;
          printf("Found a chair, navigation_state: %i\n", navigation_state);

          for(int i=0; i<4; i++) {
            target_location_odom[i] = retrieve_chair_data.legs[i];
            printf("target_location_odom, x: %f y: %f\n", target_location_odom[i].x, target_location_odom[i].y);
          }
          target_center_odom = retrieve_chair_data.center[0];
          //transfer_to_odom(target_location_hokuyo, 4, target_location_odom);
        } else {
          // wait for decision
          printf("count: %i\n", retrieve_chair_data.count);
        }
      }
    }

    //printf("found_target: %i, navigation_state: %i, status.status: %i\n", found_target, navigation_state, status.status);


    
    if(found_target == 1) {
      // if (tables.count >= 1) {
      //   compute_table_orientation(tables, 0);
      // }
      printf("main loop\n");
      //use_backleg_info();
      //printf("backleg_center x: %f    y: %f\n", backleg_center.x, backleg_center.y);
      if(seqeunce[sequence_count] == 'c') {
        check_if_exist(0);
        printf("Check existence done\n");
      } else if(seqeunce[sequence_count] == 't') {
        check_if_exist(1);
        printf("Check existence done\n");
      }

      switch(navigation_state) {
        case 1: {
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_yaw_dir = 1;
          // if(data.count == 0) {
          //   printf("count = 0, use last_value\n");
          //   drive_settings.pose.theta = last_value;    // frame correction, corrected to reference odom
          // } else {
          //   drive_settings.pose.theta = backleg_gradient;    // highlight
          //   last_value =  drive_settings.pose.theta ;
          // }
          // drive_settings.pose.theta = backleg_gradient;    // highlight
          drive_settings.pose.theta = backleg_gradient;
          printf("theta: %f\n", drive_settings.pose.theta);
          drive_settings.state = navigation_state;
          drive_pose_pub.publish(drive_settings);
          printf("drive status: %i, navigation_state: %i\n", status.status, navigation_state);
          if(status.status == 1) {
            navigation_state = 2;
            printf("drive_robot return %i, Going to state %i\n", status.status, navigation_state);
            state_1_end_x = current_pose.pose.pose.position.x;
            state_1_end_y = current_pose.pose.pose.position.y;
          }
          //printf("gradient: %f, atan(gradient): %lf, current_pose.theta: %f, atan(theta): %f\n", drive_settings.pose.theta, atan(drive_settings.pose.theta), current_pose.theta, atan(current_pose.theta));
          // if(fabsf(drive_settings.pose.theta) < 0.001) {
          //   drive_settings.stop = 1;
          //   drive_settings.enable_yaw_dir = 0;
          //   navigation_state = 2;
          //   drive_settings.state = navigation_state;
          //   drive_pose_pub.publish(drive_settings);
          //   printf("Going to state %i...\n", navigation_state);
          //   status.status = 0;
          // }
          break;
        }
        case 2: {
          printf("navigation_state 2\n");
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_y_dir = 1;
          drive_settings.type = 0;
          //drive_settings.pose.y = compute_side_movement() + current_pose.y;    // frame correction, corrected to reference odom
          // if(data.count == 0) {
          //   printf("count = 0, use last_value\n");
          //   drive_settings.pose.y = last_value;    // frame correction, corrected to reference odom
          // } else {
          //   drive_settings.pose.y = data.centerGravity[1];    // frame correction, corrected to reference odom
          //   last_value =  data.centerGravity[1];
          // }
          drive_settings.pose.y = compute_movement(0, 0);
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);
          if(status.status == 2) {
            drive_settings.stop = 1;
            drive_settings.enable_y_dir = 0;
            
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            printf("navigation_state: %i, stop driving...\n", navigation_state);
            std_msgs::Int8 command;
            //ros::Duration(2.0).sleep();
            if(platform_status.data == -1) {
              command.data = 0;
              lifting_command_pub.publish(command);
            } else {
              command.data = 5;    // dropping
              lifting_command_pub.publish(command);
            }
            if(platform_status.data == 5) {
              printf("got feeback\n");
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              command.data = 3;    // keep plate
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              navigation_state = 3;
              status.status = 0;
              platform_status.data = 0;
              state_2_end_x = current_pose.pose.pose.position.x;
              state_2_end_y = current_pose.pose.pose.position.y;
            }
          }
          break;
        }
        case 3: {
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          //drive_settings.pose.x = compute_movement(1, 0.3); // table
          drive_settings.pose.x = compute_movement(1, 0.75); // chair
          // if(tables.count != 0) {
          //   drive_settings.pose.x = current_pose.x + tables.center[0].x - 0.8;
          //   last_x_cmd = drive_settings.pose.x;
          // } else {
          //   drive_settings.pose.x = last_x_cmd;
          //   //drive_settings.pose.x = current_pose.x + backleg.x - 0.5;
          // }
          // if(data.count == 0) {
          //   drive_settings.stop = 1;
          //   drive_settings.enable_x_dir = 0;
          //   drive_settings.state = navigation_state;
          //   drive_pose_pub.publish(drive_settings);
          //   printf("Can't see four legs...\n");
          // }
          printf("pose.x: %f\n", drive_settings.pose.x);
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);
          if(status.status == 3) {
            drive_settings.stop = 1;
            drive_settings.enable_x_dir = 0;
            
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            printf("navigation_state: %i, stop driving...\n", navigation_state);

            std_msgs::Int8 command;
            //ros::Duration(2.0).sleep();
            command.data = 4;    // extend plate
            lifting_command_pub.publish(command);
            if(platform_status.data == 4) {
              //command.data = 6;    // lifting
              command.data = 7;    // lifting
              lifting_command_pub.publish(command);
              printf("wait for 15 seconds\n");
              ros::Duration(15.0).sleep();
              navigation_state = 4;
              status.status = 0;
              platform_status.data = 0;
              
            }
          }
          break;
        }
        case 4: {
          printf("currently at state 4, status.status: %i\n", status.status);
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          drive_settings.pose.x = target_center_odom.x + 0.5;    // move to 1 meter away from center
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);
          if(status.status == 4) {
            drive_settings.stop = 1;
            drive_settings.enable_y_dir = 0;
            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            printf("navigation_state: %i, stop driving...\n", navigation_state);
            status.status = 0;
            std_msgs::Int8 command;
            command.data = 5;    // dropping
            lifting_command_pub.publish(command);
            if(platform_status.data == 5) {
              printf("got feeback\n");
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              command.data = 3;    // keep plate
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              navigation_state = 5;
              status.status = 0;
              platform_status.data = 0;
            }

          }
          break;
        }
        case 5: {
          fyp2017_18::drive_message drive_settings;
          drive_settings.enable_x_dir = 1;
          // if(tables.count == 0) {
          //   drive_settings.pose.x = current_pose.x - 0.8;    // use with cautious, check coordinates
          //   last_x_cmd = drive_settings.pose.x;
          // } else {
          //   drive_settings.pose.x = last_x_cmd;
          //   drive_settings.pose.x = current_pose.x - 0.8;
          //   //drive_settings.pose.x = current_pose.x + backleg.x - 0.5;
          // }
          drive_settings.pose.x = target_center_odom.x - 1;
          printf("pose.x: %f\n", drive_settings.pose.x);
          drive_settings.state = navigation_state; 
          drive_pose_pub.publish(drive_settings);
          if(status.status == 5) {
            drive_settings.stop = 1;
            drive_settings.enable_x_dir = 0;

            drive_settings.state = navigation_state;
            drive_pose_pub.publish(drive_settings);
            printf("navigation_state: %i, stop driving...\n", navigation_state);
            std_msgs::Int8 command;
            command.data = 7;    // keeping
            lifting_command_pub.publish(command);
            if(platform_status.data == 7) {
              printf("got feeback\n");
              command.data = 0;    // stop
              lifting_command_pub.publish(command);
              ros::Duration(2.0).sleep();
              navigation_state = 6;
              status.status = 0;
              platform_status.data = 0;
            }
          }
          break;
        }
        case 6: {
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
      fyp2017_18::drive_message drive_settings;
      drive_settings.stop = 1;
      drive_pose_pub.publish(drive_settings);
      navigation_state = 0;
      printf("navigation_state: %i, stop driving...\n", navigation_state);
    }

    printf("main loop finished\n");
    double end = ros::Time::now().toSec();
    printf("main while loop used time: %f\n", end - begin); 

    r.sleep();
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
