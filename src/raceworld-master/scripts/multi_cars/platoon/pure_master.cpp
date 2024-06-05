#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "raceworld_master/status.h"
#include "tf/tf.h"
#include <unistd.h>
#include <queue>

#define PI 3.1415926535

ros::Publisher slave_vel1,slave_vel2,slave_vel3,slave_vel4;
ackermann_msgs::AckermannDriveStamped vel_msg1,vel_msg2,vel_msg3,vel_msg4;
std::string leader_name;
std::string follower1;
std::string follower2;
std::string follower3;
std::string follower4;
const double thresh_distance = 0.2;

nav_msgs::Odometry target_point1,target_point2,target_point3,target_point4;
std::queue<nav_msgs::Odometry> *ldmsg_queue1,*ldmsg_queue2,*ldmsg_queue3,*ldmsg_queue4;
int isformation;

nav_msgs::Odometry ldmsg,f1msg,f2msg,f3msg,f4msg,msg1,msg2,msg3,msg4,msg5;
bool pose_flag = false;

//Function declerations to move
void follow (nav_msgs::Odometry,nav_msgs::Odometry,nav_msgs::Odometry,nav_msgs::Odometry);
//Call back function decleration
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg); 
void statusCallBack(const raceworld_master::status::ConstPtr & status_msg);
void poseCallback1(const nav_msgs::Odometry::ConstPtr& msg);
void poseCallback2(const nav_msgs::Odometry::ConstPtr& msg);
void poseCallback3(const nav_msgs::Odometry::ConstPtr& msg);
void poseCallback4(const nav_msgs::Odometry::ConstPtr& msg);
void poseCallback5(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char** argv)
{
  ldmsg_queue1 = new std::queue<nav_msgs::Odometry>;
  ldmsg_queue2 = new std::queue<nav_msgs::Odometry>;
  ldmsg_queue3 = new std::queue<nav_msgs::Odometry>;
  ldmsg_queue4 = new std::queue<nav_msgs::Odometry>;
  ros::init(argc, argv, "follower1");

  ros::NodeHandle node;

  ros::Subscriber platoon_status = node.subscribe("/status", 1, statusCallBack);
  ros::Subscriber pose1 = node.subscribe("/deepracer1/base_pose_ground_truth", 10, poseCallback1);
  ros::Subscriber pose2 = node.subscribe("/deepracer2/base_pose_ground_truth", 10, poseCallback2);
  ros::Subscriber pose3 = node.subscribe("/deepracer3/base_pose_ground_truth", 10, poseCallback3);
  ros::Subscriber pose4 = node.subscribe("/deepracer4/base_pose_ground_truth", 10, poseCallback4);
  ros::Subscriber pose5 = node.subscribe("/deepracer5/base_pose_ground_truth", 10, poseCallback5);
  
  ros::Rate rate(100.0);
  while (ros::ok())
  {
    ros::spinOnce();
    if(follower1!="")
    {
      slave_vel1 =
      node.advertise<ackermann_msgs::AckermannDriveStamped>(follower1+"/ackermann_cmd_mux/output", 100);
      slave_vel2 =
      node.advertise<ackermann_msgs::AckermannDriveStamped>(follower2+"/ackermann_cmd_mux/output", 100);
      slave_vel3 =
      node.advertise<ackermann_msgs::AckermannDriveStamped>(follower3+"/ackermann_cmd_mux/output", 100);
      slave_vel4 =
      node.advertise<ackermann_msgs::AckermannDriveStamped>(follower4+"/ackermann_cmd_mux/output", 100);

      vel_msg1.header.stamp = ros::Time::now();
      vel_msg1.header.frame_id = follower1+"/base_link";
      vel_msg2.header.stamp = ros::Time::now();
      vel_msg2.header.frame_id = follower2+"/base_link";
      vel_msg3.header.stamp = ros::Time::now();
      vel_msg3.header.frame_id = follower3+"/base_link";
      vel_msg4.header.stamp = ros::Time::now();
      vel_msg4.header.frame_id = follower4+"/base_link";
      if(pose_flag)
      {
          ldmsg = msg1;
          f1msg = msg2;
          f2msg = msg3;
          f3msg = msg4;
          f4msg = msg5;
          ldmsg_queue1->push(ldmsg);
          ldmsg_queue2->push(f1msg);
          ldmsg_queue3->push(f2msg);
          ldmsg_queue4->push(f3msg);
          double distance1 = sqrt((target_point1.pose.pose.position.x - f1msg.pose.pose.position.x) * (target_point1.pose.pose.position.x - f1msg.pose.pose.position.x)
                                + (target_point1.pose.pose.position.y - f1msg.pose.pose.position.y) * (target_point1.pose.pose.position.y - f1msg.pose.pose.position.y));
          if(target_point1.pose.pose.position.x == 0 && target_point1.pose.pose.position.y == 0)
          target_point1 = ldmsg;
          if(distance1 < thresh_distance){
          target_point1 = ldmsg_queue1->front();
          ldmsg_queue1->pop();
          }
          else if(distance1 > 5){
            delete ldmsg_queue1;
            ldmsg_queue1 = new std::queue<nav_msgs::Odometry>;
            target_point1 = ldmsg;
          }
          double distance2 = sqrt((target_point2.pose.pose.position.x - f2msg.pose.pose.position.x) * (target_point2.pose.pose.position.x - f2msg.pose.pose.position.x)
                                + (target_point2.pose.pose.position.y - f2msg.pose.pose.position.y) * (target_point2.pose.pose.position.y - f2msg.pose.pose.position.y));
          if(target_point2.pose.pose.position.x == 0 && target_point2.pose.pose.position.y == 0)
          target_point2 = f1msg;
          if(distance2 < thresh_distance){
          target_point2 = ldmsg_queue2->front();
          ldmsg_queue2->pop();
          }
          else if(distance2 > 5){
            delete ldmsg_queue2;
            ldmsg_queue2 = new std::queue<nav_msgs::Odometry>;
            target_point2 = f1msg;
          }
          double distance3 = sqrt((target_point3.pose.pose.position.x - f3msg.pose.pose.position.x) * (target_point3.pose.pose.position.x - f3msg.pose.pose.position.x)
                                + (target_point3.pose.pose.position.y - f3msg.pose.pose.position.y) * (target_point3.pose.pose.position.y - f3msg.pose.pose.position.y));
          if(target_point3.pose.pose.position.x == 0 && target_point3.pose.pose.position.y == 0)
          target_point3 = f2msg;
          if(distance3 < thresh_distance){
          target_point3 = ldmsg_queue3->front();
          ldmsg_queue3->pop();
          }
          else if(distance3 > 5){
            delete ldmsg_queue3;
            ldmsg_queue3 = new std::queue<nav_msgs::Odometry>;
            target_point3 = f2msg;
          }
          double distance4 = sqrt((target_point4.pose.pose.position.x - f4msg.pose.pose.position.x) * (target_point4.pose.pose.position.x - f4msg.pose.pose.position.x)
                                + (target_point4.pose.pose.position.y - f4msg.pose.pose.position.y) * (target_point4.pose.pose.position.y - f4msg.pose.pose.position.y));
          if(target_point4.pose.pose.position.x == 0 && target_point4.pose.pose.position.y == 0)
          target_point4 = f3msg;
          if(distance4 < thresh_distance){
          target_point4 = ldmsg_queue4->front();
          ldmsg_queue4->pop();
          }
          else if(distance4 > 5){
            delete ldmsg_queue4;
            ldmsg_queue4 = new std::queue<nav_msgs::Odometry>;
            target_point4 = f3msg;
          }
          follow(target_point1,target_point2,target_point3,target_point4);
      }
    }
    rate.sleep();
  }
  return 0;
};


void follow (nav_msgs::Odometry target1,nav_msgs::Odometry target2,nav_msgs::Odometry target3,nav_msgs::Odometry target4)
{
  tf::Quaternion f1orientation,f2orientation,f3orientation,f4orientation,ldorientation;
  tf::quaternionMsgToTF(f1msg.pose.pose.orientation, f1orientation);
  tf::quaternionMsgToTF(f2msg.pose.pose.orientation, f2orientation);
  tf::quaternionMsgToTF(f3msg.pose.pose.orientation, f3orientation);
  tf::quaternionMsgToTF(f4msg.pose.pose.orientation, f4orientation);
  tf::quaternionMsgToTF(ldmsg.pose.pose.orientation, ldorientation);
  double delta1 = atan2(target1.pose.pose.position.y-f1msg.pose.pose.position.y,target1.pose.pose.position.x-f1msg.pose.pose.position.x);
  double delta2 = atan2(target2.pose.pose.position.y-f2msg.pose.pose.position.y,target2.pose.pose.position.x-f2msg.pose.pose.position.x);
  double delta3 = atan2(target3.pose.pose.position.y-f3msg.pose.pose.position.y,target3.pose.pose.position.x-f3msg.pose.pose.position.x);
  double delta4 = atan2(target4.pose.pose.position.y-f4msg.pose.pose.position.y,target4.pose.pose.position.x-f4msg.pose.pose.position.x);
  if(delta1 < 0){
    delta1 += 2 * PI;
  }
  if(delta2 < 0){
    delta2 += 2 * PI;
  }
  if(delta3 < 0){
    delta3 += 2 * PI;
  }
  if(delta4 < 0){
    delta4 += 2 * PI;
  }
  double roll,pitch,yaw1,yaw2,yaw3,yaw4;
  tf::Matrix3x3(f1orientation).getRPY(roll, pitch, yaw1);
  tf::Matrix3x3(f2orientation).getRPY(roll, pitch, yaw2);
  tf::Matrix3x3(f3orientation).getRPY(roll, pitch, yaw3);
  tf::Matrix3x3(f4orientation).getRPY(roll, pitch, yaw4);
  double gamma1 = yaw1;
  double gamma2 = yaw2;
  double gamma3 = yaw3;
  double gamma4 = yaw4;
  if(gamma1 < 0){
    gamma1 += 2 * PI;
  }
  if(gamma2 < 0){
    gamma2 += 2 * PI;
  }
  if(gamma3 < 0){
    gamma3 += 2 * PI;
  }
  if(gamma4 < 0){
    gamma4 += 2 * PI;
  }
  double theta1 = delta1 - gamma1;
  double theta2 = delta2 - gamma2;
  double theta3 = delta3 - gamma3;
  double theta4 = delta4 - gamma4;
  if(theta1 > PI){
    theta1 -= 2 * PI;
  }
  if(theta1 < -PI){
    theta1 += 2 * PI;
  }
  if(theta2 > PI){
    theta2 -= 2 * PI;
  }
  if(theta2 < -PI){
    theta2 += 2 * PI;
  }
  if(theta3 > PI){
    theta3 -= 2 * PI;
  }
  if(theta3 < -PI){
    theta3 += 2 * PI;
  }
  if(theta4 > PI){
    theta4 -= 2 * PI;
  }
  if(theta4 < -PI){
    theta4 += 2 * PI;
  }
  // ROS_INFO("%f",delta1);
  
  double r1 = sqrt(pow(ldmsg.pose.pose.position.x-f1msg.pose.pose.position.x, 2) +
                                  pow(ldmsg.pose.pose.position.y-f1msg.pose.pose.position.y, 2));
  double r2 = sqrt(pow(f1msg.pose.pose.position.x-f2msg.pose.pose.position.x, 2) +
                                  pow(f1msg.pose.pose.position.y-f2msg.pose.pose.position.y, 2));
  double r3 = sqrt(pow(f2msg.pose.pose.position.x-f3msg.pose.pose.position.x, 2) +
                                  pow(f2msg.pose.pose.position.y-f3msg.pose.pose.position.y, 2));
  double r4 = sqrt(pow(f3msg.pose.pose.position.x-f4msg.pose.pose.position.x, 2) +
                                  pow(f3msg.pose.pose.position.y-f4msg.pose.pose.position.y, 2));
  double k = 1.0;
  if(r1 > 0.4)
  {
    vel_msg1.drive.speed = 0.3 * r1;
  }else
  {
    vel_msg1.drive.speed = 0.05;
  }
  if(r2 > 0.4)
  {
    vel_msg2.drive.speed = 0.3 * r2;
  }else
  {
    vel_msg2.drive.speed = 0.05;
  }
  if(r3 > 0.4)
  {
    vel_msg3.drive.speed = 0.3 * r3;
  }else
  {
    vel_msg3.drive.speed = 0.05;
  }
  if(r4 > 0.4)
  {
    vel_msg4.drive.speed = 0.3 * r4;
  }else
  {
    vel_msg4.drive.speed = 0.05;
  }
  vel_msg1.drive.steering_angle = k * theta1;
  vel_msg2.drive.steering_angle = k * theta2;
  vel_msg3.drive.steering_angle = k * theta3;
  vel_msg4.drive.steering_angle = k * theta4;
  slave_vel1.publish(vel_msg1);
  slave_vel2.publish(vel_msg2);
  slave_vel3.publish(vel_msg3);
  slave_vel4.publish(vel_msg4);
}

void statusCallBack(const raceworld_master::status::ConstPtr & status_msg)
{
  leader_name = status_msg->leader;
  isformation = status_msg->formation;
  follower1 = status_msg->follower1;
  follower2 = status_msg->follower2;
  follower3 = status_msg->follower3;
  follower4 = status_msg->follower4;
}

void poseCallback1(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_flag = true;
  msg1.child_frame_id = msg->child_frame_id;
  msg1.header = msg->header;
  msg1.pose = msg->pose;
  msg1.twist = msg->twist;
}

void poseCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_flag = true;
  msg2.child_frame_id = msg->child_frame_id;
  msg2.header = msg->header;
  msg2.pose = msg->pose;
  msg2.twist = msg->twist;
}

void poseCallback3(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_flag = true;
  msg3.child_frame_id = msg->child_frame_id;
  msg3.header = msg->header;
  msg3.pose = msg->pose;
  msg3.twist = msg->twist;
}

void poseCallback4(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_flag = true;
  msg4.child_frame_id = msg->child_frame_id;
  msg4.header = msg->header;
  msg4.pose = msg->pose;
  msg4.twist = msg->twist;
}

void poseCallback5(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_flag = true;
  msg5.child_frame_id = msg->child_frame_id;
  msg5.header = msg->header;
  msg5.pose = msg->pose;
  msg5.twist = msg->twist;
}
