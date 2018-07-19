#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>

// headers for combination function
#include <algorithm>
#include <iostream>
#include <string>

// self define message header
#include <fyp2017_18/table_info.h>
#include <fyp2017_18/cluster_info.h>

#define PI 3.141595654
#define EPS 0.000001

// on actual robot is 1, on simulator is 0
#define ROBOT 0

using namespace std;

// publisher
ros::Publisher table_cluster_pub;
ros::Publisher table_legs_pub;
ros::Publisher table_CG_pub;
ros::Publisher cluster_info_pub;
ros::Publisher detected_table_data_pub;

struct cluster_group {
  float x[4];
  float y[4];
};

struct table_info {
  geometry_msgs::Point legs[4];
  geometry_msgs::Point center;
};

struct comb {
  int index[4];  
};

comb * cluster_list_index_comb;    // to store all combinations of cluters

float distances[6];
float table_reference[] = {0.66, 1.1, 1.28};
int length_freq[] = {2, 2, 2};
float allowance = 0.03;

int match_num = 0;

float angle = PI / 2;    // 90 degree
geometry_msgs::Point possible_points[4];

int factorial(int n) {
  /*
    Recursive function to calculate factorial of n
  */
  int ret = 1;
  for(int i = 1; i <= n; ++i)
      ret *= i;
  return ret;
}

void sort_cluster_by_y(geometry_msgs::Point list[], int number_of_elements) {
  /*
    Sort the array of geometry_msgs::Point in descending order of y value
    The sort result is updated directly to list[]. This function returns none.
  */
  geometry_msgs::Point temp;

  for(int i=0; i<number_of_elements; i++) {
    for(int j=i+1; j<number_of_elements; j++) {
      if(list[i].y < list[j].y) {    // "<" for descending, ">" for ascending
        temp = list[i];
        list[i] = list[j];
        list[j] = temp;
      }
    }
  }
}

void sort_ascending(int list[], int number_of_elements) {
  /*
    Sort an array of integer in ascending order
    The sort result is updated directly to list[]. This function returns none.
  */
  int temp;

  for(int i=0; i<number_of_elements; i++) {
    for(int j=i+1; j<number_of_elements; j++) {
      if(list[i] > list[j]) {    // "<" for descending, ">" for ascending
        temp = list[i];
        list[i] = list[j];
        list[j] = temp;
      }
    }
  }
}

float max_value(float array[]) {
  /*
    This function returns the largest value in the float array
  */
  float max = array[0];

  for(int i=1; i<sizeof(array)/sizeof(array[0]); i++) {
    if(array[i] > max) max = array[i];
  }
  return max;
}

float min_value(float array[]) {
  /*
    This function returns the smallest value in the float array
  */
  float min = array[0];

  for(int i=1; i<sizeof(array)/sizeof(array[0]); i++) {
    if(array[i] < min) min = array[i];
  }
  return min;
}

int check_distance(geometry_msgs::Point origin, float radius_r, geometry_msgs::Point target) {
  /*
    Calculate the distance between the origin and target point,
    compare the computed distance to the reference

    In other words, check if the target point lies on the circumference
    of the circle of radius r and center origin. 

    general solution of quadratic equation a*x^2 +b*x + c = 0 is used.

          -b ± sqrt(b^2 -4ac)                                                    
    x = -----------------------
                  2a

    The function returns 1 if the solution and the target point are close enough,
    otherwise, it return 0
  */

  float error = radius_r * sin(6 * PI / 180.0);    // range of error between the calculated point and target point
  float solution_1, solution_2;

  // constants a, b, c for the general solution
  float constant_a = 1;
  float constant_b = -2 * origin.x;
  float constant_c = pow(origin.x, 2) + pow(target.y, 2) - 2 * origin.y * target.y + pow(origin.y, 2) - pow(radius_r, 2);

  solution_1 = (-constant_b + sqrt(pow(constant_b, 2) - 4 * constant_a * constant_c)) / (2 * constant_a);
  solution_2 = (-constant_b - sqrt(pow(constant_b, 2) - 4 * constant_a * constant_c)) / (2 * constant_a);

  if(abs(solution_1 - target.x) < error) return 1;
  if(abs(solution_2 - target.x) < error) return 1;

  return 0;
}

float gradient(geometry_msgs::Point point_1, geometry_msgs::Point point_2) {
  /*
    calculate gradient between two points, return gradient in radian
  */
  float value = (point_2.y - point_1.y) / (point_2.x - point_1.x);
  if(value < 0) return (PI + atan(value));

  return atan(value);
}

void guess_points(geometry_msgs::Point point_1, geometry_msgs::Point point_2, float length) {
  /*
    Calculate the gradient of the previously identified side
    the line equation of adjacent sides are deduced based on the angles between them.
    The angle is specified in global variable <angle>.

    The remaining possible legs is calculated based on the known length of the adjacent sides.
    An assumption is made that the object (tables or chairs) are symmetrical viewed from 
    the identified side

    This function does not return. It updates value to global variable <possible_points>.
  */

  float alpha = gradient(point_1, point_2);    // gradient of identified side in radian
  float theta = angle;    // angle between adjacent side and identified side

  float gradient_1 = tan(alpha + theta);    // gradient of adjacent side
  float gradient_2 = tan(alpha + (PI - theta));

  // line extended from point_1

  float y_intercept = point_1.y - gradient_1 * point_1.x;    // y-intercept of adjacent side

  possible_points[0].x = point_1.x + sqrt(pow(length, 2) / (pow(gradient_1, 2) + 1));
  possible_points[0].y = gradient_1 * possible_points[0].x + y_intercept;
  possible_points[1].x = point_1.x - sqrt(pow(length, 2) / (pow(gradient_1, 2) + 1));
  possible_points[1].y = gradient_1 * possible_points[1].x + y_intercept;

  // line extended from point_2

  y_intercept = point_2.y - gradient_2 * point_2.x;

  possible_points[2].x = point_2.x + sqrt(pow(length, 2) / (pow(gradient_2, 2) + 1));
  possible_points[2].y = gradient_2 * possible_points[2].x + y_intercept;
  possible_points[3].x = point_2.x - sqrt(pow(length, 2) / (pow(gradient_2, 2) + 1));
  possible_points[3].y = gradient_2 * possible_points[3].x + y_intercept;

}

int find_index(geometry_msgs::Point array[], geometry_msgs::Point reference[], int array_length, int reference_length, int * store) {
  /*
    Check elements in <array> if they match with any one in <reference>,
    index of the matching element in reference will be passed to <store>

    This function returns 1 if all elements in <reference> are found in <array>,
    else returns 0
  */
  float error = 0.1;  // range of error
  int k = 2;

  for(int i=0; i<array_length; i++) {
    for(int j=0; j<reference_length; j++) {
      if(abs(array[i].x - reference[j].x) < error) {
        if(abs(array[i].y - reference[j].y) < error) {
          *(store + k) = j;
          k++;
        }
      }
    }
    if(k == 4) {
      return 1;
    }
  }

  return 0;
}


int combination(int N, int K) {
  /*
    Refer from:
    http://www.cplusplus.com/forum/beginner/128023/

    To generate index combinations of clusters
    Update cluster_list_index_comb, return total number of combinations
  */
  std::string bitmask(K, 1);    // K leading 1's
  bitmask.resize(N, 0);    // N-K trailing 0's

  int count = 0, index = 0;
  int total = factorial(N) / factorial(K) / factorial(N-K);

  free(cluster_list_index_comb);
  cluster_list_index_comb = (struct comb *) malloc (total * sizeof(struct comb));
  if (cluster_list_index_comb == NULL) exit(1);

  // print integers and permute bitmask
  do {
      for (int i = 0; i < N; ++i) {    // [0..N-1] integers
          if (bitmask[i]) {
            cluster_list_index_comb[count].index[index] = i;
            index++;
          }
      }
      count++;
      index = 0;
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  return total;    // num of combinations
}


void compute_distance(cluster_group target) {
  /*
    Compute the distances between points in the cluster group

    This function does not return. It updates the global variable <distances>
  */
  distances[0] = sqrt(pow(target.x[0] - target.x[1], 2) + pow(target.y[0] - target.y[1], 2));
  distances[1] = sqrt(pow(target.x[0] - target.x[2], 2) + pow(target.y[0] - target.y[2], 2));
  distances[2] = sqrt(pow(target.x[0] - target.x[3], 2) + pow(target.y[0] - target.y[3], 2));
  distances[3] = sqrt(pow(target.x[1] - target.x[2], 2) + pow(target.y[1] - target.y[2], 2));
  distances[4] = sqrt(pow(target.x[1] - target.x[3], 2) + pow(target.y[1] - target.y[3], 2));
  distances[5] = sqrt(pow(target.x[2] - target.x[3], 2) + pow(target.y[2] - target.y[3], 2));
}

geometry_msgs::Point compute_CG(geometry_msgs::Point target[], int index[]) {
  /*
    Compute the center of gravity of the cluster group

    This function returns the <center_gravity>.
  */
  geometry_msgs::Point center_gravity;
  int count = 4;

  for(int i=0; i<4; i++) {
    center_gravity.x += target[index[i]].x / count;
    center_gravity.y += target[index[i]].y / count;
  }

  return center_gravity;
}

void popout_elements(geometry_msgs::Point * array, int * index_to_delete, int array_length, int delete_elements) {
  /*
    Remove the desired elements in <array>

    This function does not return. It updates the variable pointed by <array>
  */
  geometry_msgs::Point results[array_length];
  
  int j = 0;
  int results_index = 0;

  for(int i=0; i<array_length; i++) {
    if(i == index_to_delete[j]) {
      j++;
      continue;
    }
    results[results_index] = array[i];
    results_index++;
  }

  for(int i=0; i<delete_elements; i++) {
    results[results_index++] = array[index_to_delete[i]];
  }

  for(int i=0; i<array_length; i++) {
    array[i] = results[i];
  }
}

int match_reference() {
  /*
    Compare the computed distances with the reference dimensions
    Return 1 if all match, else return 0
  */
  int count[] = {0, 0, 0};
  for(int i=0; i<6; i++) {
    for(int j=0; j<3; j++) {
      if(abs(distances[i] - table_reference[j]) < allowance) {
        count[j]++;
        break;
      }
    }
  }

  for(int i=0; i<3; i++) {
    if(count[i] != length_freq[i]) {
      return 0;
    } 
  }

  return 1;
}

void publish_table_legs_marker(table_info detected[], int count) {
  /*
    Configure and publish table legs' marker

    This function does not return
  */
  visualization_msgs::Marker table_legs_marker;    // marker

  // formatting marker

  // frame name of laser scanner
  #if ROBOT==0
    table_legs_marker.header.frame_id = "hokuyo";
  #elif ROBOT==1
    table_legs_marker.header.frame_id = "laser";
  #endif
  table_legs_marker.header.stamp = ros::Time::now();

  table_legs_marker.ns = "table legs";
  table_legs_marker.id = 1;

  table_legs_marker.type = visualization_msgs::Marker::POINTS;

  table_legs_marker.action = visualization_msgs::Marker::ADD;

  table_legs_marker.pose.position.x = 0;
  table_legs_marker.pose.position.y = 0;
  table_legs_marker.pose.position.z = 0;
  table_legs_marker.pose.orientation.x = 0.0;
  table_legs_marker.pose.orientation.y = 0.0;
  table_legs_marker.pose.orientation.z = 0.0;
  table_legs_marker.pose.orientation.w = 1.0;

  table_legs_marker.scale.x = 0.2;
  table_legs_marker.scale.y = 0.2;
  table_legs_marker.scale.z = 0.2;

  table_legs_marker.color.r = 1.0f;    // red marker
  table_legs_marker.color.g = 0.0f;
  table_legs_marker.color.b = 0.0f;
  table_legs_marker.color.a = 1.0;


  for(int i=0; i<count; i++) {
    for(int j=0; j<4; j++) {
      table_legs_marker.points.push_back(detected[i].legs[j]);
    }
  }

  table_legs_pub.publish(table_legs_marker);    // publish CG

}

void publish_table_CG_marker(table_info detected[], int count) {
  /*
    Configure and publish table CG's marker

    This function does not return
  */
  visualization_msgs::Marker table_CG_marker;    // marker

  // formatting marker

  // frame name of laser scanner
  #if ROBOT==0
    table_CG_marker.header.frame_id = "hokuyo";
  #elif ROBOT==1
    table_CG_marker.header.frame_id = "laser";
  #endif
  table_CG_marker.header.stamp = ros::Time::now();

  table_CG_marker.ns = "table CG";
  table_CG_marker.id = 2;

  table_CG_marker.type = visualization_msgs::Marker::POINTS;

  table_CG_marker.action = visualization_msgs::Marker::ADD;

  table_CG_marker.pose.position.x = 0;
  table_CG_marker.pose.position.y = 0;
  table_CG_marker.pose.position.z = 0;
  table_CG_marker.pose.orientation.x = 0.0;
  table_CG_marker.pose.orientation.y = 0.0;
  table_CG_marker.pose.orientation.z = 0.0;
  table_CG_marker.pose.orientation.w = 1.0;

  table_CG_marker.scale.x = 0.2;
  table_CG_marker.scale.y = 0.2;
  table_CG_marker.scale.z = 0.2;

  table_CG_marker.color.r = 1.0f;
  table_CG_marker.color.g = 0.7f;    // light red marker
  table_CG_marker.color.b = 0.7f;
  table_CG_marker.color.a = 1.0;

  for(int i=0; i<count; i++) {
    table_CG_marker.points.push_back(detected[i].center);
  }

  table_CG_pub.publish(table_CG_marker);    // publish CG

}

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
    if(laser_msg->ranges[i] > 5) {
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

    scan_angle_current += scan_angle_increment;
  }

  scan_angle_current = scan_angle_min;    // reset start scan angle

  /* clustering (segmentation) */

  float distance;    // distance between any two points
  float threshold;
  int count, point_num_threshold;
  int cluster_num;

  count = 1;
  cluster_num = 0;
  threshold = 0.2;
  point_num_threshold = 30;

  visualization_msgs::Marker cluster;   // marker
  geometry_msgs::Point cluster_coor;    // cluster coordinate
  fyp2017_18::cluster_info cluster_info_toPub;

  // formatting cluster marker

  // frame name of laser scanner
  #if ROBOT==0
    cluster.header.frame_id = "hokuyo";
  #elif ROBOT==1
    cluster.header.frame_id = "laser";
  #endif
  cluster.header.stamp = ros::Time::now();

  cluster.ns = "cluster";
  cluster.id = 0;

  cluster.type = visualization_msgs::Marker::POINTS;

  cluster.action = visualization_msgs::Marker::ADD;

  cluster.pose.position.x = 0;
  cluster.pose.position.y = 0;
  cluster.pose.position.z = 0;
  cluster.pose.orientation.x = 0.0;
  cluster.pose.orientation.y = 0.0;
  cluster.pose.orientation.z = 0.0;
  cluster.pose.orientation.w = 1.0;

  cluster.scale.x = 0.2;
  cluster.scale.y = 0.2;
  cluster.scale.z = 0.2;

  cluster.color.r = 0.0f;
  cluster.color.g = 0.0f;
  cluster.color.b = 1.0f;    // blue marker
  cluster.color.a = 1.0;
  

  for(int i=0; i<scan_ray_total; i++) {
    distance = sqrt(pow(cartesian_coor[i+1].x - cartesian_coor[i].x, 2) + pow(cartesian_coor[i+1].y - cartesian_coor[i].y, 2));    // Pythagoras' theorem
    if(distance < threshold) {    // within threshold
      count++;
      continue;
    } else {
      if(count < 3) {    // min num of points to form a cluster
        count = 1;
        continue;
      } else {
          if(count < point_num_threshold) {
            for(int j=0; j<count; j++) {
              cluster_coor.x += cartesian_coor[i-j].x;
              cluster_coor.y += cartesian_coor[i-j].y;
              cluster_coor.z = 0;
            }
            cluster_coor.x = cluster_coor.x / count;
            cluster_coor.y = cluster_coor.y / count;

            if ((abs(cluster_coor.x) < 0.01) && (abs(cluster_coor.y) < 0.01)) {
              // ignore cluster at (0,0)
              count = 1;
              cluster_coor.x = 0;
              cluster_coor.y = 0;
              continue;
            }

            cluster.points.push_back(cluster_coor);
            cluster_info_toPub.points.push_back(cluster_coor);

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
  }

  cluster_info_toPub.count = cluster_num;

  table_cluster_pub.publish(cluster);    // publish cluster markers
  cluster_info_pub.publish(cluster_info_toPub);    // publish clusters' coordinate data
  printf("Total number of cluster: %i\n", cluster_num);

  /* 
    Take any 4 from the clusters, compute distances between them
    Compare the distances with the reference
  */
  
  geometry_msgs::Point cluster_list[cluster_num];    // to store all clusters
  geometry_msgs::Point * p1 = cluster_list;
  int table_legs_cluster_index[4];
  int * p2 = table_legs_cluster_index;

  for(int i=0; i<cluster_num; i++) {
    cluster_list[i] = cluster.points[i];    // retrive info from cluster markers
  }

  sort_cluster_by_y(cluster_list, cluster_num);

  float min_side = 0.66;    // shortest side is chosen
  float side_length = 1.1;    // adjacent to shortest side
  table_info detected_table[10];    // to store the coordinates of legs of the identified table
                                    // assume it has maximum of 10, this is an array of struct
  fyp2017_18::table_info detected_table_info;
  
  match_num = 0;    // number of tables identified

  printf("Begin to search for tables from clusters\n");
  for(int i=0; i<(cluster_num-1); i++) {    // take any two clusters and check the distance
    if(cluster_num < 4) {
      // if available clusters to be iterated less than 4, cannot form a table/chair
      break;
    }
    for(int j=i+1; j<cluster_num; j++) {
      if(check_distance(cluster_list[i], min_side, cluster_list[j])) {    // check if it matches with the smallest side
        guess_points(cluster_list[i], cluster_list[j], side_length);
        *(p2 + 0) = i;    // store the identified two table legs
        *(p2 + 1) = j;
        if(find_index(possible_points, cluster_list, 4, cluster_num, p2)) {    // determining the remaining two legs
          sort_ascending(table_legs_cluster_index, 4);
          
          for(int k=0; k<4; k++) { 
            detected_table[match_num].legs[k].x = cluster_list[table_legs_cluster_index[k]].x;
            detected_table[match_num].legs[k].y = cluster_list[table_legs_cluster_index[k]].y;
            detected_table_info.legs.push_back(detected_table[match_num].legs[k]);
          }
          detected_table[match_num].center = compute_CG(cluster_list, table_legs_cluster_index);
          detected_table_info.center.push_back(detected_table[match_num].center);

          match_num++;    // table found

          popout_elements(p1, p2, cluster_num, 4);    // exclude the 4 points that are identified as a table
          cluster_num -= 4;    // reduce the clusters to be iterate
          i = 0;    // restart the iteration
          break;
        }
      }
    }
  }

  printf("%i table(s) found!\n", match_num);
  publish_table_legs_marker(detected_table, match_num);    // publish table legs marker
  publish_table_CG_marker(detected_table, match_num);      // publish table CG marker

  detected_table_info.count = match_num;
  detected_table_data_pub.publish(detected_table_info);    // publish to tables' coordinate data

  /* 
    The commented code below is previous detection method.
    Get all possible combination of 4 points from identified clusters
    Compute and compare the distances between the points
  */

  /*
  for(int i=0; i<cluster_num; i++) {
    cluster_list[i] = cluster.points[i];    // retrive info from cluster markers
  }

  cluster_group current_cluster_comb;    // to store selected cluster group
  table_info detected_table[10];

  int iterate = combination(cluster_num, 4);    // total combination
  match_num = 0;

  for(int i=0; i<iterate; i++) {
    for(int j=0; j<4; j++) {
      current_cluster_comb.x[j] = cluster_list[cluster_list_index_comb[i].index[j]].x;
      current_cluster_comb.y[j] = cluster_list[cluster_list_index_comb[i].index[j]].y;
    }
    compute_distance(current_cluster_comb);
    if(match_reference()) {    // store the details if identified as a table
      for(int j=0; j<4; j++) {
        detected_table[match_num].legs[j].x = cluster_list[cluster_list_index_comb[i].index[j]].x;
        detected_table[match_num].legs[j].y = cluster_list[cluster_list_index_comb[i].index[j]].y;
      }
      detected_table[match_num].center = compute_CG(current_cluster_comb);

      match_num++;
    }
  }

  printf("%i table(s) found!\n", match_num);
  publish_table_marker(detected_table, match_num);
  */

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle n;

  table_cluster_pub = n.advertise<visualization_msgs::Marker>("visualize_cluster", 10);
  cluster_info_pub = n.advertise<fyp2017_18::cluster_info>("cluster_data", 10);
  table_legs_pub = n.advertise<visualization_msgs::Marker>("visualize_table_legs", 10);
  table_CG_pub = n.advertise<visualization_msgs::Marker>("visualize_table_CG", 10);
  detected_table_data_pub = n.advertise<fyp2017_18::table_info>("table_data", 10);

  // To subscribe the data from Hokuyo Laser Scanning Rangefinder

  // frame name of laser scanner
  #if ROBOT==0
    ros::Subscriber laser_front_sub = n.subscribe("/mybot/laser/scan", 10, laser_scan_front);
  #elif ROBOT==1
    ros::Subscriber laser_front_sub = n.subscribe("/scan", 10, laser_scan_front);
  #endif

  ros::spin();

  return 0;
}