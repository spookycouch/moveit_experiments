#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

const double PI = 3.142;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "plan_arm_torsooo");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();






    // THIS IS WHAT WE ARE DOING TODAY AS A GROUP FUN TIMES
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // make the table
    moveit_msgs::CollisionObject object;
    object.header.frame_id = "base_footprint";
    object.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.913;
    primitive.dimensions[1] = 0.913;
    primitive.dimensions[2] = 0.82;

    geometry_msgs::Pose box_pose;
    box_pose.position.x = 1;
    box_pose.position.y = 0;
    box_pose.position.z = 0.41;
    box_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box_pose);
    object.operation = object.ADD;

    planning_scene_interface.applyCollisionObject(object);
    ros::Duration(2.0).sleep();




    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position.x = 0.876036 - 0.15;
    goal_pose.pose.position.y = -0.010633;
    goal_pose.pose.position.z = 0.9;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,0);

    group_arm_torso.setPlannerId("SBLKConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(goal_pose);

    group_arm_torso.getEndEffectorLink();
    group_arm_torso.getPlanningFrame();
    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group_arm_torso.setPlanningTime(5.0);
    group_arm_torso.plan(plan);

    group_arm_torso.move();
    spinner.stop();
}