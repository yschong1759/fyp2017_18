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
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  retrieve_cluster_data = *clusters_msg;
}

void get_table_data(const fyp2017_18::table_info::ConstPtr& tables_msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  retrieve_table_data = *tables_msg;
}

void get_chair_data(const fyp2017_18::table_info::ConstPtr& chairs_msg) {
  /*
    Callback funtion: store data from the topic to a global variable

    This function does not return.
  */
  retrieve_chair_data = *chairs_msg;
}


void transformPoint(const tf::TransformListener& listener){
  /*
    Get the transform relationship between odom and laser scanning rangefinder.

    Convert the cluster coordinates from laser scanning rangefinder frame 
    to odom frame. 

    Publish the transformed coorfinates to respective topics. 

    Refer to ROS Tutorials for details.
    Writing a tf listener (C++)
    http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29

    This function does not return. 
  */

  geometry_msgs::PointStamped cluster_data_before;
  geometry_msgs::PointStamped table_data_before;
  geometry_msgs::PointStamped chair_data_before;
  
  geometry_msgs::PointStamped cluster_data_after;
  geometry_msgs::PointStamped table_data_after;
  geometry_msgs::PointStamped chair_data_after;

  try {
    geometry_msgs::Point transform;

    #if ROBOT==0
      listener.waitForTransform("/hokuyo", "/odom",
                                ros::Time::now(), ros::Duration(2.0));
    #elif ROBOT==1
      listener.waitForTransform("/laser", "/odom",
                                ros::Time::now(), ros::Duration(2.0));
    #endif
    
    for(int i=0; i<retrieve_cluster_data.count; i++) {
      #if ROBOT==0
        cluster_data_before.header.frame_id = "hokuyo";
      #elif ROBOT==1
        cluster_data_before.header.frame_id = "laser";
      #endif
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
      #if ROBOT==0
        table_data_before.header.frame_id = "hokuyo";
      #elif ROBOT==1
        table_data_before.header.frame_id = "laser";
      #endif
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
      #if ROBOT==0
        table_data_before.header.frame_id = "hokuyo";
      #elif ROBOT==1
        table_data_before.header.frame_id = "laser";
      #endif
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

    for(int i=0; i<retrieve_chair_data.count * 4; i++) {    // chair legs
      #if ROBOT==0
        chair_data_before.header.frame_id = "hokuyo";
      #elif ROBOT==1
        chair_data_before.header.frame_id = "laser";
      #endif
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

    for(int i=0; i<retrieve_chair_data.count; i++) {    // chair center
      #if ROBOT==0
        chair_data_before.header.frame_id = "hokuyo";
      #elif ROBOT==1
        chair_data_before.header.frame_id = "laser";
      #endif
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

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"odom\": %s", ex.what());
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "to_odom");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  ros::Subscriber cluster_data_sub = n.subscribe("/cluster_data", 10, get_cluster_data);
  ros::Subscriber table_data_sub = n.subscribe("/table_data", 10, get_table_data);
  ros::Subscriber chair_data_sub = n.subscribe("/chair_data", 10, get_chair_data);
  
  // 10 Hz
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

  cluster_data_odom_pub = n.advertise<fyp2017_18::cluster_info>("cluster_data_odom", 10);
  table_data_odom_pub = n.advertise<fyp2017_18::table_info>("table_data_odom", 10);
  chair_data_odom_pub = n.advertise<fyp2017_18::table_info>("chair_data_odom", 10);

  ros::spin();

  return 0;
}