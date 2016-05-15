#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <sstream>

  float theta;        // current angle of the robot
  float alpha;        // desired angle  (to target)
  float x;            // current x
  float y;            // current y
  float X;            // Target  X
  float Y;            // Target  Y
  float angle_Err;    // angle Error
  float distance=0;   // distance from current location to Target

  #define Kp_lin 0.3  // lineal Proportional controller CONSTANT
  #define Kp_ang 5    // angular Proportional controller CONSTANT

void Pose_Callback(const geometry_msgs::Pose::ConstPtr& Pose)      // get the current Pose
{
  theta=Pose->orientation.w;        // #### assuming 2D driving - taking only angle of Quaternion without the vector ####
  x=Pose->position.x;
  y=Pose->position.y;
}

void Target_Callback(const geometry_msgs::Pose::ConstPtr& target)  // get the target Pose
{
  X=target->position.x;
  Y=target->position.y;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "location_control");

  ros::NodeHandle n;

  ros::Publisher rosbot_pub = n.advertise<geometry_msgs::Twist>("/rosbot/speed_command", 1000);
  ros::Subscriber Pose_sub = n.subscribe("/rosbot/localization", 1000, Pose_Callback);
  ros::Subscriber Target_sub = n.subscribe("/rosbot/target", 1000, Target_Callback);

  ros::Rate loop_rate(50);
  
  geometry_msgs::Twist g_msgs;     //define Twist object (for sending velocity commands)

 
  while (ros::ok())
  {
    alpha= atan2((Y-y),(X-x));                     //angle from current orientation to Target
    angle_Err=theta-alpha;                         // angle Error
    distance= sqrtf(pow((Y-y),2) + pow((X-x),2));  // distance from current position to Target
    
    if(distance<=0.05) distance=0;                 // Tolerance
    if(distance>0.05 && distance<1) distance=1;    // define minimum velocity
    if(distance>5) distance=1;                     // define maximum velocity

    g_msgs.linear.x = distance*Kp_lin;             //determine linear velocity
    g_msgs.angular.z = -angle_Err*Kp_ang;          //determine angular velocity

    ROS_INFO("x=        %f", x);                   //
    ROS_INFO("y=        %f", y);                   //
    ROS_INFO("X=        %f", X);                   //
    ROS_INFO("Y=        %f", Y);                   // for debugging !
    ROS_INFO("alpha=    %f", alpha);               //
    ROS_INFO("theta=    %f", theta);               //
    ROS_INFO("angle_Err %f", angle_Err);           //
    ROS_INFO("distance  %f", distance);            //

    rosbot_pub.publish(g_msgs);                    // publish commands
  

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
