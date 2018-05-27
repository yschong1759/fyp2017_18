#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// include custom message header
#include <fyp2017_18/cluster_info.h>

// on actual robot is 1, on simulator is 0
#define ROBOT 0

// publisher
ros::Publisher cluster_pub;
ros::Publisher total_pub;
ros::Publisher conversion_pub;

void laser_scan_front(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
  /* 
    visualize environment
    convert laser scan reading to Cartesian coordinate
  */

  float scan_angle_increment = laser_msg->angle_increment;
  float scan_angle_min = laser_msg->angle_min;
  float scan_angle_max = laser_msg->angle_max;
  float scan_time_increment = laser_msg->time_increment;
  float scan_time = laser_msg->scan_time;
  float scan_range_min = laser_msg->range_min;
  float scan_range_max = laser_msg->range_max;
  float scan_angle_current = scan_angle_min;    // start from the min angle for cartesian conversion

  int scan_ray_total = (scan_angle_max - scan_angle_min) / scan_angle_increment + 1;
  
  geometry_msgs::Point cartesian_coor[scan_ray_total];    // to store cartesian conversion results

  for(int i=0; i<=scan_ray_total; i++) {
    if(laser_msg->ranges[i] > 4) {
      // to filter simulation range bug or
      // to ignore points that are too far
      cartesian_coor[i].x = 0;
      cartesian_coor[i].y = 0;
      cartesian_coor[i].z = 0;
    } else {
      cartesian_coor[i].x = laser_msg->ranges[i] * cos(scan_angle_current);
      cartesian_coor[i].y = laser_msg->ranges[i] * sin(scan_angle_current);
      cartesian_coor[i].z = 0;
    }

    scan_angle_current += scan_angle_increment;    // angle increment
  }

  scan_angle_current = scan_angle_min;    // reset start scan angle

  /* clustering (segmentation) */

  float distance;    // distance between any two points
  float threshold;
  int count, point_num_threshold;
  int cluster_num;

  threshold = 0.2;
  count = 1;
  point_num_threshold = 30;
  cluster_num = 0;

  visualization_msgs::Marker cluster_marker;    // marker
  fyp2017_18::cluster_info output_info;    // message to be published
  geometry_msgs::Point cluster_coor;    // cluster coordinate
  
  // iteration starts
  for(int i=0; i<scan_ray_total; i++) {
    distance = sqrt(pow(cartesian_coor[i+1].x - cartesian_coor[i].x, 2) + pow(cartesian_coor[i+1].y - cartesian_coor[i].y, 2));    // Pythagoras' theorem
    if(distance < threshold) {    // within threshold
      count++;
      continue;
    } else {
      if(count < 3) {    // min num of points to form a cluster
        count = 1;
        continue;
      } else if(count < point_num_threshold) {
          for(int j=0; j<count; j++) {
            cluster_coor.x += cartesian_coor[i-j].x;
            cluster_coor.y += cartesian_coor[i-j].y;
            cluster_coor.z = 0;
          }
          cluster_coor.x = cluster_coor.x / count;
          cluster_coor.y = cluster_coor.y / count;

          if ((abs(cluster_coor.x) < 0.01) && (abs(cluster_coor.y) < 0.01)) {
            // ignore cluster near to (0,0)
            count = 1;
            cluster_coor.x = 0;
            cluster_coor.y = 0;
            continue;
          }

          cluster_marker.points.push_back(cluster_coor);
          output_info.points.push_back(cluster_coor);

          cluster_num++;
          count = 1;
          cluster_coor.x = 0;
          cluster_coor.y = 0;
        } else {    // too many points
          count = 1;
          continue;
        }
    }
  }

  // formatting cluster marker
  #if ROBOT==0
    cluster_marker.header.frame_id = "hokuyo";
  #elif ROBOT==1
    cluster_marker.header.frame_id = "laser";
  #endif
  cluster_marker.header.stamp = ros::Time::now();

  cluster_marker.ns = "cluster_marker";
  cluster_marker.id = 1;

  cluster_marker.type = visualization_msgs::Marker::POINTS;

  cluster_marker.action = visualization_msgs::Marker::ADD;

  cluster_marker.pose.position.x = 0;
  cluster_marker.pose.position.y = 0;
  cluster_marker.pose.position.z = 0;
  cluster_marker.pose.orientation.x = 0.0;
  cluster_marker.pose.orientation.y = 0.0;
  cluster_marker.pose.orientation.z = 0.0;
  cluster_marker.pose.orientation.w = 1.0;

  cluster_marker.scale.x = 0.2;
  cluster_marker.scale.y = 0.2;
  cluster_marker.scale.z = 0.2;

  cluster_marker.color.r = 0.0f;
  cluster_marker.color.g = 0.0f;
  cluster_marker.color.b = 1.0f;    // blue marker
  cluster_marker.color.a = 1.0;

  output_info.count = cluster_num;

  cluster_pub.publish(cluster_marker);    // publish cluster markers
  total_pub.publish(output_info);    // publish number of clusters

  printf("Total number of cluster: %i\n", cluster_num);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "clustering");
  ros::NodeHandle n;

  cluster_pub = n.advertise<visualization_msgs::Marker>("visualize_cluster", 10);
  total_pub = n.advertise<fyp2017_18::cluster_info>("cluster", 10);
  //conversion_pub = n.advertise<visualization_msgs::Marker>("converted", 10);

  #if ROBOT==0
    ros::Subscriber laser_front_sub = n.subscribe("/mybot/laser/scan", 10, laser_scan_front);
  #elif ROBOT==1
    ros::Subscriber laser_front_sub = n.subscribe("/laser", 10, laser_scan_front);
  #endif

  ros::spin();

  return 0;
}