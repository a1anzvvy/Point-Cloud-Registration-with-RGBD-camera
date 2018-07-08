#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Char.h>

int state = 0;

void KeyboardCallback(const std_msgs::CharConstPtr& msg)
{
  if (msg->data == 't'){
    state = 1;
  }else if(msg->data == 'y'){
    state = 0;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Subscriber sub = node.subscribe("/Keyboard_Input", 1000, KeyboardCallback);


  ros::Rate rate(10.0);
  while (node.ok()){
    if (state == 1){
      transform.setOrigin( tf::Vector3(-0.25403,0.344606,0.6853) );
      transform.setRotation( tf::Quaternion(-0.12533, -0.196402, -0.400985, 0.885963) );
      printf("Coordinate Change! state = %d\n", state);
    }
    else{
      transform.setOrigin( tf::Vector3(0.4052, 0.2566, 0.5957) );
      transform.setRotation( tf::Quaternion(0.187607, 0.619297, 0.743992, 0.166588) );
      printf("state = %d\n", state);
    }
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_link"));
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
};