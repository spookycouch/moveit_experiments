#include <jeff_moveit/simple_grasps.h>


void simple_grasps(std::vector<moveit_msgs::Grasp> &grasps, geometry_msgs::PointStamped point, geometry_msgs::Vector3 size, geometry_msgs::Vector3 retreat) {
    if (size.y < GRASP_MAX)
        grasps.push_back(front_90(point));
    if (size.z < GRASP_MAX)
        grasps.push_back(front_180(point));
    if (size.x < GRASP_MAX)
        grasps.push_back(top_90(point));

    // standard across all simple grasps
    for (std::vector<moveit_msgs::Grasp>::iterator it = grasps.begin(); it != grasps.end(); ++it) {
        it->pre_grasp_posture.joint_names.resize(2);
        it->pre_grasp_posture.joint_names[0] = "gripper_left_finger_joint";
        it->pre_grasp_posture.joint_names[1] = "gripper_right_finger_joint";
        it->pre_grasp_posture.points.resize(1);
        it->pre_grasp_posture.points[0].positions.resize(2);
        it->pre_grasp_posture.points[0].positions[0] = 0.05;
        it->pre_grasp_posture.points[0].positions[1] = 0.05;
        it->pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

        // note: the rosparams
        //     /gripper_controller/constraints/gripper_left_finger_joint/goal
        //     /gripper_controller/constraints/gripper_left_finger_joint/goal
        // must be sufficiently large to reach this goal; set in
        //     /opt/pal/erbium/share/tiago_controller_configuration/config/pal-gripper_joint_trajectory_controllers.yaml
        it->grasp_posture.joint_names.resize(2);
        it->grasp_posture.joint_names[0] = "gripper_left_finger_joint";
        it->grasp_posture.joint_names[1] = "gripper_right_finger_joint";
        it->grasp_posture.points.resize(1);
        it->grasp_posture.points[0].positions.resize(2);
        it->grasp_posture.points[0].positions[0] = 0.00;
        it->grasp_posture.points[0].positions[1] = 0.00;
        it->grasp_posture.points[0].time_from_start = ros::Duration(0.5);

        it->post_grasp_retreat.direction.header.frame_id = "base_footprint";
        it->post_grasp_retreat.direction.vector = retreat;
        it->post_grasp_retreat.min_distance = 0.1;
        it->post_grasp_retreat.desired_distance = 0.2;
    }
}


// Front approach, rotate the gripper by 90 degrees.
// imagine holding an upright pole from the front.
//
moveit_msgs::Grasp front_90(geometry_msgs::PointStamped point) {
    moveit_msgs::Grasp grasp;
    grasp.grasp_pose.header.frame_id = point.header.frame_id;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,0,0); // coffee
    grasp.grasp_pose.pose.position.x = point.point.x - GRIPPER_FINGER_LENGTH;
    grasp.grasp_pose.pose.position.y = point.point.y;
    grasp.grasp_pose.pose.position.z = point.point.z;

    geometry_msgs::Vector3 approach;
    approach.x = 0.3; approach.y = 0.0; approach.z = 0.0;

    grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint";
    grasp.pre_grasp_approach.direction.vector = approach;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.9;

    return grasp;
}


// Front approach, rotate the gripper by 180 degrees.
// good for grabbing door handles from the front.
//
moveit_msgs::Grasp front_180(geometry_msgs::PointStamped point) {
    moveit_msgs::Grasp grasp;
    grasp.grasp_pose.header.frame_id = point.header.frame_id;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI,0,0); // door
    grasp.grasp_pose.pose.position.x = point.point.x - GRIPPER_LENGTH;
    grasp.grasp_pose.pose.position.y = point.point.y;
    grasp.grasp_pose.pose.position.z = point.point.z;

    geometry_msgs::Vector3 approach;
    approach.x = 0.3; approach.y = 0.0; approach.z = 0.0;

    grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint";
    grasp.pre_grasp_approach.direction.vector = approach;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.9;

    return grasp;
}


// Front approach, rotate the gripper by 90 degrees.
// imagine holding an upright pole from the front.
//
moveit_msgs::Grasp top_90(geometry_msgs::PointStamped point) {
    moveit_msgs::Grasp grasp;
    grasp.grasp_pose.header.frame_id = point.header.frame_id;
    grasp.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(PI/2,PI/2,0); // coffee
    grasp.grasp_pose.pose.position.x = point.point.x;
    grasp.grasp_pose.pose.position.y = point.point.y;
    grasp.grasp_pose.pose.position.z = point.point.z + GRIPPER_LENGTH;

    geometry_msgs::Vector3 approach;
    approach.x = 0.0; approach.y = 0.0; approach.z = -0.3;

    grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint";
    grasp.pre_grasp_approach.direction.vector = approach;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.9;

    return grasp;
}