#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

#define ROBOT 1

ros::Publisher Marker_pub;
ros::Publisher Table_pub;
ros::Publisher Laser_pub;
ros::Publisher Backleg_pub;

sensor_msgs::LaserScan Msg_laser;
sensor_msgs::LaserScan Msg_laser1;

vector< vector<int> > V_combResult;

float F_clus_centre[50][2];     // {x, y}
float F_table_pose[5][2];       // {center, 2 pts making length, 2 pts making width}
int I_length_maker;
float f_state = 0;
float check;

float F_table_dim[3] = {0.69, 1.1, 1.29};
float F_table_dim_thres = 0.2;

float computeDist(int pt_index1, int pt_index2)
{
  float x1, x2, y1, y2;

  x1 = F_clus_centre[pt_index1][0];
  x2 = F_clus_centre[pt_index2][0];
  y1 = F_clus_centre[pt_index1][1];
  y2 = F_clus_centre[pt_index2][1];

  return sqrt( ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) );
}

bool checkTable(float dist1, float dist2, float dist3, float dist4, float dist5, float dist6)
{
  float f_table_dim[3] = {0.69, 1.1, 1.29};
  float f_table_dim_thres = 0.15;
  int i_dim_check[3] = {0, 0, 0};
  float f_dist[6];

  f_dist[0] = dist1;
  f_dist[1] = dist2;
  f_dist[2] = dist3;
  f_dist[3] = dist4;
  f_dist[4] = dist5;
  f_dist[5] = dist6;

  for (int i = 0; i < 6; i++)
  {
    if ((f_dist[i] > f_table_dim[0]-f_table_dim_thres) && (f_dist[i] < f_table_dim[0]+f_table_dim_thres)) 
    {
      I_length_maker = i;
      i_dim_check[0]++;

      if (i == 0) check = 1;

    }
    else if ((f_dist[i] > f_table_dim[1]-f_table_dim_thres) && (f_dist[i] < f_table_dim[1]+f_table_dim_thres))
    {
      i_dim_check[1]++;
      if ((i == 1) && (check ==1)) check = 2;

    } 
    else if ((f_dist[i] > f_table_dim[2]-f_table_dim_thres) && (f_dist[i] < f_table_dim[2]+f_table_dim_thres)) i_dim_check[2]++;   
  }

  ROS_INFO("check1 = %d  check2 = %d   check3 = %d", i_dim_check[0], i_dim_check[1], i_dim_check[2]);
  if ((i_dim_check[0] == 2) && (i_dim_check[1] == 2) && (i_dim_check[2] == 2))
  //if (i_dim_check[0]+i_dim_check[1]+i_dim_check[2] == 4)
    {
      return 1;
    }
  else return 0;
}

int combination(int n, int k)
{
    std::string bitmask(k, 1); // K leading 1's
    bitmask.resize(n, 0); // N-K trailing 0's

    if (k > n) return 0;
    V_combResult.clear();

    do
    {
      vector<int> v_pts;
      for (int i = 0; i < n; ++i) // [0..N-1] integers
      {
        if (bitmask[i]) v_pts.push_back(i);
      }
      V_combResult.push_back(v_pts);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    return V_combResult.size();
}

void laserCallback1(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  ROS_INFO("Lasercallback1");
  int i_comb_ct = 0;
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 60;
  float f_clus_sum = 0;
  int i_no_readings = 0;
  float f_range_thres = 1.5;
  
  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.2;
  float angle_excl = 15;  //degree

  visualization_msgs::Marker points;

#ifdef ROBOT  
  points.header.frame_id = "odom";
#else
  points.header.frame_id = "scan";
#endif

  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  i_no_readings = 1 + ((Msg_laser.angle_max - Msg_laser.angle_min)/Msg_laser.angle_increment);
  Msg_laser.ranges.resize(i_no_readings);

  
  angle_excl = 12*2*3.1416/360;
  //change to index
  int index_start = angle_excl/ Msg_laser.angle_increment;
  
  ROS_INFO("Index_start = %d", index_start);

  // for (int i = index_start; i < (i_no_readings-index_start); ++i)
  for (int i = index_start; i < (i_no_readings-index_start); ++i)
  {
    // convert range sensor reading to x-y coordinates
    f_pre_pose[0] = Msg_laser.ranges[i-1] * sin((i-1) * Msg_laser.angle_increment);
    f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1] * cos((i-1) * Msg_laser.angle_increment);
    f_cur_pose[0] = Msg_laser.ranges[i] * sin((i) * Msg_laser.angle_increment);
    f_cur_pose[1] = (-1) * Msg_laser.ranges[i] * cos((i) * Msg_laser.angle_increment);

    f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0], 2) + pow(f_cur_pose[1]-f_pre_pose[1], 2));

    while(f_dist_diff < f_diff_thres)
    {
      // if next point is within the threshold
      i_pt_ct++;
      if (i_no_readings < i+i_pt_ct) break;
      f_pre_pose[0] = Msg_laser.ranges[i-1+i_pt_ct] * sin((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1+i_pt_ct] * cos((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[0] = Msg_laser.ranges[i+i_pt_ct] * sin((i+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[1] = (-1) * Msg_laser.ranges[i+i_pt_ct] * cos((i+i_pt_ct) * Msg_laser.angle_increment);
      
      // take the adjacent point and update the difference
      f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));
      
    }
    //ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    
    if (i_pt_ct == 0)
    {
      // if point count equals to zero,
      // no cluster of point
      f_clus_sum = Msg_laser.ranges[i];
      F_clus_centre[i_clus_ct][0] = Msg_laser.ranges[i-1] * sin((i) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * Msg_laser.ranges[i-1] * cos((i) * Msg_laser.angle_increment);
    } 

    else
    {
      // cluster of point detected
      for (int y = 0; y < (i_pt_ct); ++y)
      {
        f_clus_sum += Msg_laser.ranges[i+y];
      }

      // calculating center point of the cluster
      float ave;
      ave = f_clus_sum / (i_pt_ct);
      F_clus_centre[i_clus_ct][0] = ave * sin((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * ave * cos((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
    }

    float check_range;
    check_range = sqrt(pow(F_clus_centre[i_clus_ct][0], 2) + pow(F_clus_centre[i_clus_ct][1], 2));
    if ((check_range > f_range_thres) || (i_pt_ct > i_no_threspt))
    {
      // if exceed threshold, reject the cluster
      F_clus_centre[i_clus_ct][0] = F_clus_centre[i_clus_ct][1] = 0;
    }
    else
    {
      i_clus_ct++;
    }
    
    i = i + i_pt_ct;
    i_pt_ct = 0;
    f_clus_sum = 0;
  }

  // based on the number of clusters detected,
  // calculated the number of combination of 2
  i_comb_ct = combination(i_clus_ct, 2);
  ROS_INFO("LaserCallback1: i_comb_ct = %d", i_comb_ct);

  for (int i = 0; i < i_comb_ct; i++)
  {
    float f_dist;
    bool b_backleg_check = 0;

    ROS_INFO("LaserCallback1: %d %d", V_combResult[i][0], V_combResult[i][1]);

    f_dist = computeDist(V_combResult[i][0], V_combResult[i][1]);
    ROS_INFO("LaserCallback1: f_dist = %f", f_dist);

    if ((f_dist > F_table_dim[1] - F_table_dim_thres) && (f_dist < F_table_dim[1] + F_table_dim_thres))
    {
      ROS_INFO("Backleg found!");
      geometry_msgs::Point p;

      // to finsh the center of the two clusters of the back leg
      p.x = F_clus_centre[V_combResult[i][0]][0];
      p.y = F_clus_centre[V_combResult[i][0]][1];
      points.points.push_back(p);

      p.x = F_clus_centre[V_combResult[i][1]][0];
      p.y = F_clus_centre[V_combResult[i][1]][1];
      points.points.push_back(p);

      // calculate the midpoint of the two backlegs
      p.x = (F_clus_centre[V_combResult[i][0]][0] + F_clus_centre[V_combResult[i][1]][0]) / 2;
      p.y = (F_clus_centre[V_combResult[i][0]][1] + F_clus_centre[V_combResult[i][1]][1]) / 2;
      points.points.push_back(p);
      Backleg_pub.publish(points);

      b_backleg_check = 1;
    }

    if (b_backleg_check == 1) break;
  }

}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  int i_comb_ct = 0;
  int i_pt_ct = 0, i_clus_ct = 0;
  int i_no_threspt = 30;
  float f_clus_sum = 0;
  int i_no_readings = 0;
  float f_range_thres = 3.0;
  
  float f_pre_pose[2] = {0, 0};   // {x, y}
  float f_cur_pose[2] = {0, 0};   // {x, y}
  float f_dist_diff = 0;
  float f_diff_thres = 0.35;

  Msg_laser.angle_increment = msg->angle_increment;
  Msg_laser.angle_min = msg->angle_min;
  Msg_laser.angle_max = msg->angle_max;
  Msg_laser.time_increment = msg->time_increment;
  Msg_laser.scan_time = msg->scan_time;
  Msg_laser.range_min = msg->range_min;
  Msg_laser.range_max = msg->range_max;

#ifdef ROBOT  
  Msg_laser.header.frame_id = "odom";
#else
  Msg_laser.header.frame_id = "laser";
 #endif 

  visualization_msgs::Marker points;

#ifdef ROBOT  
  points.header.frame_id = "odom";
#else
  points.header.frame_id = "laser";
#endif

  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  i_no_readings = 1 + ((Msg_laser.angle_max - Msg_laser.angle_min)/Msg_laser.angle_increment);
  Msg_laser.ranges.resize(i_no_readings);

  for (int i = 0; i < i_no_readings; ++i)
  {
    Msg_laser.ranges[i] = msg->ranges[i];

    if(isnan(Msg_laser.ranges[i]) || isinf(Msg_laser.ranges[i]))
    {
      Msg_laser.ranges[i] = 5;
    }
  }
  Laser_pub.publish(Msg_laser);

  for (int i = 1; i < i_no_readings; ++i)
  {
    f_pre_pose[0] = Msg_laser.ranges[i-1] * sin((i-1) * Msg_laser.angle_increment);
    f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1] * cos((i-1) * Msg_laser.angle_increment);
    f_cur_pose[0] = Msg_laser.ranges[i] * sin((i) * Msg_laser.angle_increment);
    f_cur_pose[1] = (-1) * Msg_laser.ranges[i] * cos((i) * Msg_laser.angle_increment);

    f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0], 2) + pow(f_cur_pose[1]-f_pre_pose[1], 2));

    while(f_dist_diff < f_diff_thres)
    {
      i_pt_ct++;
      if (i_no_readings < i+i_pt_ct) break;
      f_pre_pose[0] = Msg_laser.ranges[i-1+i_pt_ct] * sin((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_pre_pose[1] = (-1) * Msg_laser.ranges[i-1+i_pt_ct] * cos((i-1+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[0] = Msg_laser.ranges[i+i_pt_ct] * sin((i+i_pt_ct) * Msg_laser.angle_increment);
      f_cur_pose[1] = (-1) * Msg_laser.ranges[i+i_pt_ct] * cos((i+i_pt_ct) * Msg_laser.angle_increment);
      f_dist_diff = sqrt(pow(f_cur_pose[0]-f_pre_pose[0] ,2) + pow(f_cur_pose[1]-f_pre_pose[1] ,2));
      
    }
    //ROS_INFO("i - %d   i_pt_ct - %d", i, i_pt_ct);
    
    if (i_pt_ct == 0)
    {
      f_clus_sum = Msg_laser.ranges[i];
      F_clus_centre[i_clus_ct][0] = Msg_laser.ranges[i-1] * sin((i) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * Msg_laser.ranges[i-1] * cos((i) * Msg_laser.angle_increment);
    } 

    else
    {
      for (int y = 0; y < (i_pt_ct); ++y)
      {
        f_clus_sum += Msg_laser.ranges[i+y];
      }
      float ave;
      ave = f_clus_sum / (i_pt_ct);
      F_clus_centre[i_clus_ct][0] = ave * sin((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
      F_clus_centre[i_clus_ct][1] = (-1) * ave * cos((i+((i_pt_ct)/2)) * Msg_laser.angle_increment);
    }

    float check_range;
    check_range = sqrt(pow(F_clus_centre[i_clus_ct][0], 2) + pow(F_clus_centre[i_clus_ct][1], 2));
    if ((check_range > f_range_thres) || (i_pt_ct > i_no_threspt))
    {
      F_clus_centre[i_clus_ct][0] = F_clus_centre[i_clus_ct][1] = 0;
    }
    else
    {
      i_clus_ct++;
    }
    
    i = i + i_pt_ct;
    i_pt_ct = 0;
    f_clus_sum = 0;
  }    


  for (int i = 0; i < i_clus_ct; ++i)
  {
    geometry_msgs::Point p;
    p.x = F_clus_centre[i][0];
    p.y = F_clus_centre[i][1];
    points.points.push_back(p);
  }
  
  Marker_pub.publish(points);
  
   i_comb_ct = combination(i_clus_ct, 4);
   ROS_INFO("Combination count = %d", i_comb_ct);

   for (int i = 0; i < i_comb_ct; i++)
   {
    ROS_INFO("i = %d", i);
    float f_dist1, f_dist2, f_dist3, f_dist4, f_dist5, f_dist6;
    bool b_table_check = 0;

    ROS_INFO("Combinations - %d %d %d %d", V_combResult[i][0], V_combResult[i][1], V_combResult[i][2], V_combResult[i][3]);
    f_dist1 = computeDist(V_combResult[i][0], V_combResult[i][1]);
    f_dist2 = computeDist(V_combResult[i][1], V_combResult[i][2]);
    f_dist3 = computeDist(V_combResult[i][2], V_combResult[i][3]);
    f_dist4 = computeDist(V_combResult[i][3], V_combResult[i][0]);
    f_dist5 = computeDist(V_combResult[i][0], V_combResult[i][2]);
    f_dist6 = computeDist(V_combResult[i][1], V_combResult[i][3]);
    ROS_INFO("f_dist1 = %f   f_dist2 = %f   f_dist3 = %f   f_dist4 = %f", f_dist1, f_dist2, f_dist3, f_dist4);
    
    b_table_check = checkTable(f_dist1, f_dist2, f_dist3, f_dist4, f_dist5, f_dist6);


    if (b_table_check == 1)
     {
      int i_pt_index[4];

      visualization_msgs::Marker table;

#ifdef ROBOT  
      table.header.frame_id = "odom";
#else
      table.header.frame_id = "laser";
#endif

      table.header.stamp = ros::Time::now();
      table.ns = "table";
      table.action = visualization_msgs::Marker::ADD;
      table.pose.orientation.w = 1.0;
      table.id = 2;
      table.type = visualization_msgs::Marker::POINTS;
      table.scale.x = 0.2;
      table.scale.y = 0.2;
      table.color.g = 0.0;
      table.color.b = 0.0;
      table.color.a = 1.0;
      table.color.r = 1.0;
      
      geometry_msgs::Point p;

      for (int y = 0; y < 4; y++)
      {
        i_pt_index[y] = V_combResult[i][y];
      } 
      p.x = p.y = p.z = 0.0;
      table.points.push_back(p); //robot position
      
      for (int y = 0; y < 4; y++)
      {
        p.x += F_clus_centre[i_pt_index[y]][0];
        p.y += F_clus_centre[i_pt_index[y]][1];
      }

      p.x = p.x / 4.0;
      p.y = p.y / 4.0;
      table.points.push_back(p);
      ROS_INFO("Centre x= %f y = %f", p.x, p.y);

      

      if (I_length_maker == 0)
      {
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
      }
      else if (I_length_maker == 1)
      {
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);

      }
      else if (I_length_maker == 2)
      {
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        if (check == 2)
        {
            p.x = 1.0;
            p.y = 0.0;
            p.z = 0.0;
        }
        table.points.push_back(p);
      }
      else if (I_length_maker == 3)
      {
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
      }
      else if (I_length_maker == 4)
      {
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
      }
      else
      {
        p.x = F_clus_centre[V_combResult[i][1]][0];
        p.y = F_clus_centre[V_combResult[i][1]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][3]][0];
        p.y = F_clus_centre[V_combResult[i][3]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][0]][0];
        p.y = F_clus_centre[V_combResult[i][0]][1];
        table.points.push_back(p);
        p.x = F_clus_centre[V_combResult[i][2]][0];
        p.y = F_clus_centre[V_combResult[i][2]][1];
        table.points.push_back(p);
      }


      ROS_INFO("Table found!");

      Table_pub.publish(table);
      check = 0;

      break;
    }
   }
}

int main(int argc, char** argv )
{
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle n;
  Marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Laser_pub = n.advertise<sensor_msgs::LaserScan>("laserpub", 10);
  Table_pub = n.advertise<visualization_msgs::Marker>("table_marker", 10);
  Backleg_pub = n.advertise<visualization_msgs::Marker>("backleg_marker", 10);
  ros::Subscriber sublaser = n.subscribe("/mybot/laser/scan", 10, laserCallback);
  ros::Subscriber sublaser1 = n.subscribe("/mybot/laser/scan", 10, laserCallback1);

  ros::spin();
}