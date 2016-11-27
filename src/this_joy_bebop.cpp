#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher reset_pub_;
  ros::Publisher snapshot_pub_;
  ros::Subscriber joy_sub_;
  
};

TeleopJoy::TeleopJoy()
{
  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("bebop/takeoff", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("bebop/land", 1);
  reset_pub_ = nh_.advertise<std_msgs::Empty>("bebop/reset", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0.5 * joy->axes[1];
//l_scale_*joy->axes[1];
  twist.linear.y = 0.5 * joy->axes[0];
//l_scale_*joy->axes[0];
  twist.linear.z = 0.5 * joy->axes[7];
//l_scale_*joy->axes[7]; 
  twist.angular.z = 0.5 * joy->axes[3];
//a_scale_*joy->axes[3];
  vel_pub_.publish(twist);
  
  bool takeoff_pressed = joy->buttons.at(3);
  bool land_pressed = joy->buttons.at(1);
  bool reset_pressed = joy->buttons.at(2);
  
  std_msgs::Empty myMsg;
  
  if (takeoff_pressed){
	takeoff_pub_.publish(myMsg);
  }
  if (land_pressed){
	land_pub_.publish(myMsg);
  }
  if (reset_pressed){
	reset_pub_.publish(myMsg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  ros::spin();
}
