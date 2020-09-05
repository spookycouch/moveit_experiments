#include <ros/ros.h>
#include "jeff_moveit/AddCollisionBox.h"
#include "jeff_moveit/AddOccupancyGrid.h"
#include "jeff_moveit/PlanningSceneControl.h"
#include "jeff_moveit/Pick.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// function defs
bool add_collision_box(jeff_moveit::AddCollisionBox::Request &req, jeff_moveit::AddCollisionBox::Response &res);
bool move();


// global variable defs
const double PI = 3.142;
tf::TransformListener* listener;
moveit::planning_interface::MoveGroupInterface* group_arm_torso;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;


/* 
 * Add occupany grid
 * 
 * filters cloud to only include points bounded by filter_min and filter_max
 * downsamples cloud s.t. each point is a grid_size centimetre cube
 *
 * creates an occupancy grid in the planning scene via a collision object
 * including each point captured in the filtered/downsampled cloud
 *
 */
bool add_occupancy_grid(jeff_moveit::AddOccupancyGrid::Request &req, jeff_moveit::AddOccupancyGrid::Response &res) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xtion_frame_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // transform to base_footprint
    pcl::fromROSMsg(req.points, *xtion_frame_cloud);
    pcl_ros::transformPointCloud(req.frame_id, *xtion_frame_cloud, *cloud, *listener);

    // remove nans
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *nan_filtered_cloud, nan_indices);
    cloud.swap(nan_filtered_cloud);

    // filter x,y,z to cube (min-max)
    pcl::PassThrough<pcl::PointXYZ> ptfilter(true);
    ptfilter.setInputCloud(cloud);
    
    ptfilter.setFilterFieldName("x");
    ptfilter.setFilterLimits(req.filter_min.x, req.filter_max.x);
    ptfilter.filter(*cloud);

    ptfilter.setFilterFieldName("y");
    ptfilter.setFilterLimits(req.filter_min.y, req.filter_max.y);
    ptfilter.filter(*cloud);

    ptfilter.setFilterFieldName("z");
    ptfilter.setFilterLimits(req.filter_min.z, req.filter_max.z);
    ptfilter.filter(*cloud);

    // create voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(req.grid_size, req.grid_size, req.grid_size);
    vg.filter(*downsampled_cloud);
    cloud.swap(downsampled_cloud);


    // apply grid as collision object in planning scene
    moveit_msgs::CollisionObject object;
    object.header.frame_id = req.frame_id;
    object.id = req.id;

    tf2::Quaternion default_rotation;
    default_rotation.setRPY(0,0,0);

    geometry_msgs::Quaternion quaternion_msg;
    tf2::convert(quaternion_msg, default_rotation);

    int count = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point_it = cloud->points.begin(); point_it != cloud->points.end(); ++point_it) {
        count += 1;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = req.box_size;
        primitive.dimensions[1] = req.box_size;
        primitive.dimensions[2] = req.box_size;

        geometry_msgs::Pose box_pose;
        box_pose.position.x = point_it->x;
        box_pose.position.y = point_it->y;
        box_pose.position.z = point_it->z;
        box_pose.orientation = quaternion_msg;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(box_pose);
    }

    object.operation = object.ADD;
    planning_scene_interface->applyCollisionObject(object);


    pcl::toROSMsg(*cloud, res.points);
    return true;
}


bool add_collision_box(jeff_moveit::AddCollisionBox::Request &req, jeff_moveit::AddCollisionBox::Response &res) {
    // make the table
    moveit_msgs::CollisionObject object;
    object.header.frame_id = req.frame_id;
    object.id = req.id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = req.size.x;
    primitive.dimensions[1] = req.size.y;
    primitive.dimensions[2] = req.size.z;

    geometry_msgs::Pose box_pose;
    box_pose.position = req.position;
    box_pose.orientation = req.orientation;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box_pose);
    object.operation = object.ADD;

    planning_scene_interface->applyCollisionObject(object);
    return true;
}


bool planning_scene_controller(jeff_moveit::PlanningSceneControl::Request &req, jeff_moveit::PlanningSceneControl::Response &res) {
    res.error = "OK";
    if (req.input.compare("CLEAR_COLLISION") == 0)
        planning_scene_interface->removeCollisionObjects(planning_scene_interface->getKnownObjectNames());
    else if (req.input.compare("CLEAR_ATTACH") == 0)
        group_arm_torso->detachObject();
    else
        res.error = "INVALID INPUT";

    return true;
}


bool move(jeff_moveit::Move::Request &req, jeff_moveit::Move::Response &res) {
    group_arm_torso->setPlannerId("SBLKConfigDefault");
    group_arm_torso->setPoseReferenceFrame("base_footprint");
    group_arm_torso->setPoseTarget(req.pose);

    group_arm_torso->getEndEffectorLink();
    group_arm_torso->getPlanningFrame();
    group_arm_torso->setStartStateToCurrentState();
    group_arm_torso->setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    group_arm_torso->setPlanningTime(5.0);
    group_arm_torso->plan(plan);

    group_arm_torso->move();
    return true;
}

bool pick(jeff_moveit::Pick::Request &req, jeff_moveit::Pick::Response &res) {
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].pre_grasp_posture.joint_names.resize(2);
    grasps[0].pre_grasp_posture.joint_names[0] = "gripper_left_finger_joint";
    grasps[0].pre_grasp_posture.joint_names[1] = "gripper_right_finger_joint";
    grasps[0].pre_grasp_posture.points.resize(1);
    grasps[0].pre_grasp_posture.points[0].positions.resize(2);
    grasps[0].pre_grasp_posture.points[0].positions[0] = 0.04;
    grasps[0].pre_grasp_posture.points[0].positions[1] = 0.04;
    grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // note: the rosparams
    //     /gripper_controller/constraints/gripper_left_finger_joint/goal
    //     /gripper_controller/constraints/gripper_left_finger_joint/goal
    // must be sufficiently large to reach this goal; set in
    //     /opt/pal/erbium/share/tiago_controller_configuration/config/pal-gripper_joint_trajectory_controllers.yaml
    grasps[0].grasp_posture.joint_names.resize(2);
    grasps[0].grasp_posture.joint_names[0] = "gripper_left_finger_joint";
    grasps[0].grasp_posture.joint_names[1] = "gripper_right_finger_joint";
    grasps[0].grasp_posture.points.resize(1);
    grasps[0].grasp_posture.points[0].positions.resize(2);
    grasps[0].grasp_posture.points[0].positions[0] = 0.00;
    grasps[0].grasp_posture.points[0].positions[1] = 0.00;
    grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    grasps[0].grasp_pose.header.frame_id = req.frame_id;
    // grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); // door
    grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,0); // coffee
    grasps[0].grasp_pose.pose.position.x = req.position.x;
    grasps[0].grasp_pose.pose.position.y = req.position.y;
    grasps[0].grasp_pose.pose.position.z = req.position.z;

    geometry_msgs::Vector3 approach, retreat;
    approach.x = 1.0; approach.y = 0.0; approach.z = 0.0;
    // retreat.x = 0.0; retreat.y = 0.0; retreat.z = -0.3; // door
    retreat.x = 0.0; retreat.y = 0.0; retreat.z = 1.0; // coffee

    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
    grasps[0].pre_grasp_approach.direction.vector.x = approach.x;
    grasps[0].pre_grasp_approach.direction.vector.y = approach.y;
    grasps[0].pre_grasp_approach.direction.vector.z = approach.z;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.9;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
    grasps[0].post_grasp_retreat.direction.vector.x = retreat.x;
    grasps[0].post_grasp_retreat.direction.vector.y = retreat.y;
    grasps[0].post_grasp_retreat.direction.vector.z = retreat.z;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.2;

    std::cout << "ERROR CODE: " << group_arm_torso->pick(req.id, grasps) << std::endl;

    return true;
}



int main(int argc, char ** argv) {
    ros::init(argc, argv, "plan_arm_torsooo");

    listener = new tf::TransformListener();
    ros::Duration(2).sleep();
    
    group_arm_torso = new moveit::planning_interface::MoveGroupInterface("arm_torso");
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    ros::AsyncSpinner spinner(0);
    ros::NodeHandle nh;
    ros::ServiceServer add_box_service = nh.advertiseService("add_collision_box", add_collision_box);
    ros::ServiceServer add_occupancy_service = nh.advertiseService("add_occupancy_grid", add_occupancy_grid);
    ros::ServiceServer planning_scene_service = nh.advertiseService("planning_scene_ctl", planning_scene_controller);
    ros::ServiceServer pick_service = nh.advertiseService("pick_object", pick);
    ros::ServiceServer move_service = nh.advertiseService("move_gripper", move);
    spinner.start();
    ros::waitForShutdown();
}
