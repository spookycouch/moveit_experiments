#ifndef JEFF_MOVEIT_SIMPLE_GRASPS
#define JEFF_MOVEIT_SIMPLE_GRASPS

#include <vector>
#include <tf/transform_datatypes.h>

#include <moveit_msgs/Grasp.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

const double PI = 3.142;
const double GRIPPER_FINGER_LENGTH = 0.165;
const double GRIPPER_LENGTH = 0.21875;
const double GRASP_MAX = 0.08;

void simple_grasps(std::vector<moveit_msgs::Grasp> &grasps, geometry_msgs::PointStamped point, geometry_msgs::Vector3 size, geometry_msgs::Vector3 retreat);
moveit_msgs::Grasp front_90(geometry_msgs::PointStamped point);
moveit_msgs::Grasp front_180(geometry_msgs::PointStamped point);
moveit_msgs::Grasp top_90(geometry_msgs::PointStamped point);
moveit_msgs::Grasp top_00(geometry_msgs::PointStamped point);
moveit_msgs::Grasp left_90(geometry_msgs::PointStamped point);
moveit_msgs::Grasp left_00(geometry_msgs::PointStamped point); // i don't see much use for this but ok

#endif