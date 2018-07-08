#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// Filters
#include <pcl/filters/passthrough.h>  
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>  

// Point Cloud Registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

// Flag to save the point cloud information
bool ifSave = 0;
ros::Publisher point2_pub;
int i = 0;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void PointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  sensor_msgs::PointCloud2 output;
  output = *msg;

  // // Transfer to PCL format
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); 
  pcl::PCLPointCloud2 cloud_filtered;     //存储滤波后的数据格式

  // 转化为PCL中的点云的数据格式
  pcl_conversions::toPCL(*msg, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;   //实例化滤波
  sor.setInputCloud(cloudPtr);                //设置输入点云
  sor.setFilterFieldName("z");             //设置过滤时所需要点云类型的z字段
  sor.setFilterLimits(0.0,1.0);           //设置在过滤字段上的范围
  // sor.setFilterLimitsNegative (true);     //设置保留范围内的还是过滤掉范围内的
  sor.setLeafSize(0.005,0.005,0.005); 
  sor.filter(cloud_filtered);              //执行滤波，保存过滤结果在cloud_filtered

  // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor_1;
  // sor_1.setInpulCloud()
  // ROS_INFO("Listening:");
  // point2_pub.publish(output);

  pcl_conversions::fromPCL(cloud_filtered, output);

  // point2_pub.publish(output);
  if (ifSave){
    // ROS_INFO("I heard: [%d]", msg->height);
    ifSave = 0;
    // Publish point cloud data
    point2_pub.publish(output);

    // Composite File Name
    char count[10];
    sprintf(count, "%d", i);
    char file_name[8];
    strcpy(file_name,"toy");
    strcat(file_name,count);
    strcat(file_name,".pcd");
    i = i+1;;
    ROS_INFO("saving file [%d]:,%s",i,file_name);

    // Turn into PointXYZ
    pcl::PointCloud<pcl::PointXYZ> cloud_saved;
    pcl::fromPCLPointCloud2(cloud_filtered, cloud_saved);

    // Save File
    pcl::io::savePCDFile (file_name, cloud_saved);
  }
}

void KeyboardCallback(const std_msgs::CharConstPtr& msg)
{
  if (msg->data == 'r'){
    ifSave  = 1;
    ROS_INFO("Publishing PointCloud2!");
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "pt2_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  point2_pub = n.advertise<sensor_msgs::PointCloud2>("Point2_keyboard", 1);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, PointCallback);
  ros::Subscriber sub1 = n.subscribe("/Keyboard_Input", 1000, KeyboardCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::spin();

  return 0;
}