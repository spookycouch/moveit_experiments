#include <ros/ros.h>
#include "jeff_moveit/AddCollisionBox.h"
#include "jeff_moveit/AddOccupancyGrid.h"
#include "jeff_moveit/PlanningSceneControl.h"
#include "jeff_moveit/Pick.h"
#include "jeff_moveit/Move.h"

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

#include <jeff_moveit/simple_grasps.h>

// function defs
bool add_collision_box(jeff_moveit::AddCollisionBox::Request &req, jeff_moveit::AddCollisionBox::Response &res);
bool move(jeff_moveit::Move::Request &req, jeff_moveit::Move::Response &res);


// global variable defs
tf::TransformListener* listener;
moveit::planning_interface::MoveGroupInterface* group_arm_torso;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;


/* 
 * Add occupany grid
 * 
 * filters cloud to only include points bounded by filter_min and filter_max
 * downsamples cloud s.t. each point is a grid_leaf_size centimetre cuboid
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
    vg.setLeafSize(req.grid_leaf_size.x, req.grid_leaf_size.y, req.grid_leaf_size.z);
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
        primitive.dimensions[0] = req.box_dimensions.x;
        primitive.dimensions[1] = req.box_dimensions.y;
        primitive.dimensions[2] = req.box_dimensions.z;

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
    res.pose = group_arm_torso->getCurrentPose();

    return true;
}


bool pick(jeff_moveit::Pick::Request &req, jeff_moveit::Pick::Response &res) {
    std::vector<moveit_msgs::Grasp> grasps;
    simple_grasps(grasps, req.point, req.size, req.retreat);
    std::cout << "ERROR CODE: " << group_arm_torso->pick(req.id, grasps) << std::endl;
    res.pose = group_arm_torso->getCurrentPose();

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
