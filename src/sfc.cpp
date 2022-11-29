#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
const double PI = 3.14159265358979;

sensor_msgs::PointCloud cloud;
nav_msgs::Odometry agent_odom, target_odom;
bool cloud_ok = false;
bool agent_ok = false;
bool target_ok = false;

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sensor_msgs::PointCloud2 cloud_dummy = *msg;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_dummy, cloud);
    cloud_ok = true;
}
void agent_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    agent_odom = *msg;
    agent_ok = true;
}
void target_odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    target_odom = *msg;
    target_ok = true;
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "collision check");
  ros::NodeHandle nh("~");
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/points", 10, cloud_cb);
  ros::Subscriber agent_odom_sub = nh.subscribe<nav_msgs::Odometry>("/agent/odom", 10, agent_odom_cb);
  // ros::Subscriber target_odom_sub = nh.subscribe<nav_msgs::Odometry>("/target/odom", 10, target_odom_cb);
  ros::Subscriber target_odom_sub = nh.subscribe<nav_msgs::Odometry>("/noisy_odom", 10, target_odom_cb);
  ros::Publisher target_detect_pub = nh.advertise<nav_msgs::Odometry>("/target_detect_odom", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedron_array", 1, true);
  int k = 20;
  double r = 0.5;
  nh.param("/agentConstraints/polygon_check", k, 20);
  nh.param("/agentConstraints/polygon_check_radius", r, 0.5);
  ros::Rate rate(20.0);
  while(ros::ok()){
    if(cloud_ok && agent_ok && target_ok){
      vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);
      vec_Vec2f obs2d;
      for(const auto& it: obs)
        obs2d.push_back(it.topRows<2>());

      vec_Vec2f path;
      Vec2f p0, p1, dummy;
      p0 << agent_odom.pose.pose.position.x,agent_odom.pose.pose.position.y;
      p1 << target_odom.pose.pose.position.x, target_odom.pose.pose.position.y;
      path.push_back(p0); 
      path.push_back(p1);

      //Using ellipsoid decomposition
      EllipsoidDecomp2D decomp_util;
      decomp_util.set_obs(obs2d);
      decomp_util.set_local_bbox(Vec2f(1, 2));
      decomp_util.dilate(path); //Set max iteration number of 10, do fix the path
      decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
      auto polys = decomp_util.get_polyhedrons();
      const auto pt_inside = (p0 + p1) / 2;
      LinearConstraint2D cs(pt_inside, polys[0].hyperplanes());
      int count = 0;
      for(size_t i = 0; i < k; i++) {
        dummy << pt_inside[0]+r*cos(2*PI*i/k), pt_inside[1]+r*sin(2*PI*i/k); 
        if(cs.inside(dummy))
          count=count+1;
      }
      if (count>k/3){
        poly_msg.header.frame_id = "odom";
        poly_pub.publish(poly_msg);
        target_detect_pub.publish(target_odom);
      }
      else{
        ROS_WARN("Cannot detect a target, count = %d", count);
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();

  return 0;
}
