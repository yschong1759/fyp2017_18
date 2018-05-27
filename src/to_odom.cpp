#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <fyp2017_18/cluster_info.h>
#include <fyp2017_18/table_info.h>


fyp2017_18::cluster_info retrieve_cluster_data;
fyp2017_18::cluster_info prepare_cluster_data;
fyp2017_18::table_info retrieve_table_data;
fyp2017_18::table_info prepare_table_data;
fyp2017_18::table_info retrieve_chair_data;
fyp2017_18::table_info prepare_chair_data;

ros::Publisher table_data_odom_pub;
ros::Publisher cluster_data_odom_pub;
ros::Publisher chair_data_odom_pub;


void get_cluster_data(const fyp2017_18::cluster_info::ConstPtr& clusters_msg) {
  retrieve_cluster_data = *clusters_msg;
}

void get_table_data(const fyp2017_18::table_info::ConstPtr& tables_msg) {
  retrieve_table_data = *tables_msg;
}

void get_chair_data(const fyp2017_18::table_info::ConstPtr& chairs_msg) {
  retrieve_chair_data = *chairs_msg;
}


void transformPoint(const tf::TransformListener& listener){
  double begin = ros::Time::now().toSec();  

  //we'll create a point in the base_laser frame that we'd like to transform to the odom frame
  geometry_msgs::PointStamped cluster_data_before;
  geometry_msgs::PointStamped table_data_before;
  geometry_msgs::PointStamped chair_data_before;
  
  geometry_msgs::PointStamped cluster_data_after;
  geometry_msgs::PointStamped table_data_after;
  geometry_msgs::PointStamped chair_data_after;
  
  cluster_data_before.header.frame_id = "laser";

  //we'll just use the most recent transform available for our simple example
  cluster_data_before.header.stamp = ros::Time();

  //just an arbitrary point in space
  cluster_data_before.point.x = 1.0;
  cluster_data_before.point.y = 0.2;
  cluster_data_before.point.z = 0.0;

  try{
    geometry_msgs::Point transform;
    listener.waitForTransform("/laser", "/odom",
                              ros::Time::now(), ros::Duration(2.0));
    
    for(int i=0; i<retrieve_cluster_data.count; i++) {
      cluster_data_before.header.frame_id = "laser";
      cluster_data_before.header.stamp = ros::Time();
      cluster_data_before.point.x = retrieve_cluster_data.points[i].x;
      cluster_data_before.point.y = retrieve_cluster_data.points[i].y;
      cluster_data_before.point.z = retrieve_cluster_data.points[i].z;
      listener.transformPoint("odom", cluster_data_before, cluster_data_after);
      printf("after cluster transform: %f %f %f\n", cluster_data_after.point.x, cluster_data_after.point.y, cluster_data_after.point.z);

      transform.x = cluster_data_after.point.x;
      transform.y = cluster_data_after.point.y;
      transform.z = cluster_data_after.point.z;

      prepare_cluster_data.points.push_back(transform);
    }
    prepare_cluster_data.count = retrieve_cluster_data.count;
    cluster_data_odom_pub.publish(prepare_cluster_data);

    for(int i=0; i<retrieve_table_data.count * 4; i++) {  // table legs
      table_data_before.header.frame_id = "laser";
      table_data_before.header.stamp = ros::Time();
      table_data_before.point.x = retrieve_table_data.legs[i].x;
      table_data_before.point.y = retrieve_table_data.legs[i].y;
      table_data_before.point.z = retrieve_table_data.legs[i].z;
      listener.transformPoint("odom", table_data_before, table_data_after);
      printf("after table transform: %f %f %f\n", table_data_after.point.x, table_data_after.point.y, table_data_after.point.z);

      transform.x = table_data_after.point.x;
      transform.y = table_data_after.point.y;
      transform.z = table_data_after.point.z;

      prepare_table_data.legs.push_back(transform);
    }
    for(int i=0; i<retrieve_table_data.count; i++) {  // table center
      table_data_before.header.frame_id = "laser";
      table_data_before.header.stamp = ros::Time();
      table_data_before.point.x = retrieve_table_data.center[i].x;
      table_data_before.point.y = retrieve_table_data.center[i].y;
      table_data_before.point.z = retrieve_table_data.center[i].z;
      listener.transformPoint("odom", table_data_before, table_data_after);
      printf("after table transform: %f %f %f\n", table_data_after.point.x, table_data_after.point.y, table_data_after.point.z);

      transform.x = table_data_after.point.x;
      transform.y = table_data_after.point.y;
      transform.z = table_data_after.point.z;

      prepare_table_data.center.push_back(transform);
    }
    prepare_table_data.count = retrieve_table_data.count;
    table_data_odom_pub.publish(prepare_table_data);

    for(int i=0; i<retrieve_chair_data.count * 4; i++) {
      chair_data_before.header.frame_id = "laser";
      chair_data_before.header.stamp = ros::Time();
      chair_data_before.point.x = retrieve_chair_data.legs[i].x;
      chair_data_before.point.y = retrieve_chair_data.legs[i].y;
      chair_data_before.point.z = retrieve_chair_data.legs[i].z;
      listener.transformPoint("odom", chair_data_before, chair_data_after);
      printf("after chair transform: %f %f %f\n", chair_data_after.point.x, chair_data_after.point.y, chair_data_after.point.z);

      transform.x = chair_data_after.point.x;
      transform.y = chair_data_after.point.y;
      transform.z = chair_data_after.point.z;

      prepare_chair_data.legs.push_back(transform);
    }
    for(int i=0; i<retrieve_chair_data.count; i++) {
      chair_data_before.header.frame_id = "laser";
      chair_data_before.header.stamp = ros::Time();
      chair_data_before.point.x = retrieve_chair_data.center[i].x;
      chair_data_before.point.y = retrieve_chair_data.center[i].y;
      chair_data_before.point.z = retrieve_chair_data.center[i].z;
      listener.transformPoint("odom", chair_data_before, chair_data_after);
      printf("after chair transform: %f %f %f\n", chair_data_after.point.x, chair_data_after.point.y, chair_data_after.point.z);

      transform.x = chair_data_after.point.x;
      transform.y = chair_data_after.point.y;
      transform.z = chair_data_after.point.z;

      prepare_chair_data.center.push_back(transform);
    }
    prepare_chair_data.count = retrieve_chair_data.count;
    chair_data_odom_pub.publish(prepare_chair_data);


    // ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
    //     laser_point.point.x, laser_point.point.y, laser_point.point.z,
    //     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"odom\": %s", ex.what());
  }

  double end = ros::Time::now().toSec();
  printf("transformPoint used time: %f\n", end - begin);  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "to_odom");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second

  ros::Subscriber cluster_data_sub = n.subscribe("/cluster_data", 10, get_cluster_data);
  ros::Subscriber table_data_sub = n.subscribe("/table_data", 10, get_table_data);
  ros::Subscriber chair_data_sub = n.subscribe("/chair_data", 10, get_chair_data);
  


  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

  cluster_data_odom_pub = n.advertise<fyp2017_18::cluster_info>("cluster_data_odom", 10);
  table_data_odom_pub = n.advertise<fyp2017_18::table_info>("table_data_odom", 10);
  chair_data_odom_pub = n.advertise<fyp2017_18::table_info>("chair_data_odom", 10);


  ros::spin();

}