#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

// self define message header
#include <fyp2017_18/entry_point.h>
#include <fyp2017_18/drive_message.h>
#include <fyp2017_18/drive_feedback.h>

#define PI 3.141595654

ros::Publisher cmd_vel_pub;
ros::Publisher drive_status;
ros::Publisher feedback_pub;

struct PID_param {
  int enabled;
  int type;
  float setpoint;    // in the context here, SP is position
  float known_error;    // adapted
  float Kp;
  float Ki;
  float Kd;
  float dt;    // dt in milliseconds
  float max_output;    // output command velocity
  float min_output;
};

struct PID_counter {    // to store variables for next iteration
  float integral_accum;
  float previous_error;
};


geometry_msgs::Pose2D current_pose;
fyp2017_18::drive_message drive_input;

// void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

//   current_pose.x = odom_msg->pose.pose.position.x;
//   current_pose.y = odom_msg->pose.pose.position.y;

//   tf::Quaternion q(
//   odom_msg->pose.pose.orientation.x,
//   odom_msg->pose.pose.orientation.y,
//   odom_msg->pose.pose.orientation.z,
//   odom_msg->pose.pose.orientation.w);
//   tf::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);

//   current_pose.theta = yaw;
// }

// float stop_x, stop_y;
// float yaw;

// void entry_point_callback(const fyp2017_18::entry_point::ConstPtr& msg) {
//   if (msg->count == 0) {
    
//   } else {
//     stop_x = msg->centerGravity[0] - 0.3;
//     yaw = -msg->lineEqParam[0];
//     printf("tangent of yaw angle: %f\n", yaw);
//   }
// }

float PID_Controller(PID_param parameter, PID_counter history, float measured_value) {

  float error;
  float derivative, output;

  if(parameter.enabled == 0) {
    printf("it is disabled\n");
    return 0;
  }

  if(parameter.type == 1) {
    error = parameter.known_error;
  } else {
    error = parameter.setpoint - measured_value;
  }
  
  history.integral_accum = history.integral_accum + error * parameter.dt;
  derivative =  (error - history.previous_error) / parameter.dt;
  output = parameter.Kp * error + parameter.Ki * history.integral_accum + parameter.Kd * derivative;
  //printf("output: %f\n", output);
  history.previous_error = error;

  if(output > parameter.max_output) return parameter.max_output;
  if(output < parameter.min_output) return parameter.min_output;

  return output;

}

void get_pose(const geometry_msgs::Pose2D::ConstPtr& pose_msg) {
  current_pose = *pose_msg;
}

void get_setpoint(const fyp2017_18::drive_message::ConstPtr& msg) {
  drive_input = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "drive_robot");
  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("odom_to_rpy", 10, get_pose);
  ros::Subscriber setpoint_sub = n.subscribe("pose_setpoint", 10, get_setpoint);
  //ros::Subscriber entry_point_sub = n.subscribe("visualize_table_entry_points", 10, entry_point_callback);

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  feedback_pub = n.advertise<fyp2017_18::drive_feedback>("drive_feedback", 10);

  float time_step = 100;    // time in milliseconds
  PID_param direction_x;
  PID_param direction_y;
  PID_param rotation_z;
  PID_counter direction_x_history;
  PID_counter direction_y_history;
  PID_counter rotation_z_history;

  direction_x_history.integral_accum = 0;
  direction_x_history.previous_error = 0;

  direction_x.enabled = drive_input.enable_x_dir;
  direction_x.setpoint = 0;
  direction_x.Kp = 0.1;
  direction_x.Ki = 0.03;
  direction_x.Kd = 0.3;
  direction_x.dt = time_step;    // milliseconds
  direction_x.max_output = 0.3;    // meter per second
  direction_x.min_output = -0.3;    // meter per second
  direction_x.type = drive_input.type;

  direction_y_history.integral_accum = 0;
  direction_y_history.previous_error = 0;

  direction_y.enabled = drive_input.enable_y_dir;
  direction_y.setpoint = 0;
  direction_y.Kp = 0.5;
  direction_y.Ki = 0.01;
  direction_y.Kd = 5;
  direction_y.dt = time_step;    // milliseconds
  direction_y.max_output = 0.3;    // meter per second
  direction_y.min_output = -0.3;    // meter per second
  direction_y.type = drive_input.type;

  rotation_z_history.integral_accum = 0;
  rotation_z_history.previous_error = 0;

  rotation_z.enabled = drive_input.enable_yaw_dir;
  rotation_z.setpoint = 0;
  rotation_z.Kp = 0.1;
  rotation_z.Ki = 0.02;
  rotation_z.Kd = 0.3;
  rotation_z.dt = time_step;    // milliseconds
  rotation_z.max_output = 0.4;
  rotation_z.min_output = -0.4;
  rotation_z.type = drive_input.type;

  ros::Rate r(1/(time_step/1000));    // in Hz

  geometry_msgs::Twist cmd_vel;
  fyp2017_18::drive_feedback control_feedback;

  while(ros::ok()) {
    if(drive_input.stop == 1) {
      printf("PID is disabled!\n");
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;

      cmd_vel_pub.publish(cmd_vel);
      r.sleep();
    }
    
    direction_x.setpoint = drive_input.pose.x;
    direction_y.setpoint = drive_input.pose.y;
    rotation_z.setpoint = drive_input.pose.theta;  // desired_pose in radian value

    direction_x.enabled = drive_input.enable_x_dir;
    direction_y.enabled = drive_input.enable_y_dir;
    rotation_z.enabled = drive_input.enable_yaw_dir;

    cmd_vel.linear.x = PID_Controller(direction_x, direction_x_history, current_pose.x);
    cmd_vel.linear.y = PID_Controller(direction_y, direction_y_history, current_pose.y);
    cmd_vel.angular.z = PID_Controller(rotation_z, rotation_z_history, current_pose.theta);

    cmd_vel_pub.publish(cmd_vel);

    printf("error: %f\n", direction_x.setpoint - current_pose.x);

    if(drive_input.state == 1) {
      if(fabsf(rotation_z.setpoint - current_pose.theta) < 0.005) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel);

        control_feedback.status = 1;
        feedback_pub.publish(control_feedback);
      }
    } else if(drive_input.state == 2) {
      if(fabsf(direction_y.setpoint - current_pose.y) < 0.01) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel);

        control_feedback.status = 2;
        feedback_pub.publish(control_feedback);
      }
    } else if(drive_input.state == 3) {
      if(fabsf(direction_x.setpoint - current_pose.x) < 0.05) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel);

        control_feedback.status = 3;
        feedback_pub.publish(control_feedback);
      }
    } else if(drive_input.state == 4) {
      if(fabsf(direction_x.setpoint - current_pose.x) < 0.05) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel);

        control_feedback.status = 4;
        feedback_pub.publish(control_feedback);
      }
    } else if(drive_input.state == 5) {
      if(fabsf(direction_x.setpoint - current_pose.x) < 0.05) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel);

        control_feedback.status = 5;
        feedback_pub.publish(control_feedback);
      }
    } else {
      control_feedback.status = 0;
      feedback_pub.publish(control_feedback);
    }


    r.sleep();
    ros::spinOnce();
  }

}