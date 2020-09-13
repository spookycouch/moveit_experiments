#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import linear_sum_assignment
from tf.transformations import quaternion_from_euler
from cmd_vels import move as cmd_vel

from jeff_segment_objects.srv import SegmentObjects, RemoveBox
from jeff_moveit.srv import AddCollisionBox, PlanningSceneControl, AddOccupancyGrid, Pick, Move
from lasr_object_detection_yolo.srv import YoloDetection

from geometry_msgs.msg import Point, Quaternion, PointStamped, Vector3, PoseStamped, Twist
from sensor_msgs.msg import PointCloud2, Image
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from math import pi



def transform_pclmsg(target_frame, pclmsg):
    try:
        trans = tf_buffer.lookup_transform(target_frame, pclmsg.header.frame_id, pclmsg.header.stamp, rospy.Duration(4.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
    
    return do_transform_cloud(pclmsg, trans)


def get_pclmsg(queue_size=5):
    # clear the buffer to get a recent pcl
    pcl_list = []

    def pcl_callback(pcl):
        pcl_list.append(pcl)
    
    pcl_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, pcl_callback)
    while not len(pcl_list) >= queue_size:
        rospy.sleep(0.1)

    pcl_sub.unregister()
    return pcl_list.pop(-1)


def frame_imgmsg_from_pclmsg(pclmsg):
    bridge = CvBridge()
    frame = np.fromstring(pclmsg.data, dtype=np.uint8)
    frame = frame.reshape(480,640,pclmsg.point_step)
    frame = frame[:,:,16:19]
    imgmsg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    return frame.copy(), imgmsg


def pclxyz_from_pclmsg(pclmsg):
    pcl = np.fromstring(pclmsg.data, dtype=np.float32)
    pcl = pcl.reshape(480,640,pclmsg.point_step/4)
    pcl = pcl[:,:,:3]
    return pcl.copy()


def play_motion(motion_name):
    # Wait for the play motion server to come up and send goal
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))

    # Create the play_motion goal and send it
    pose_goal = PlayMotionGoal()
    pose_goal.motion_name = motion_name
    pose_goal.skip_planning = True
    play_motion_client.send_goal(pose_goal)
    play_motion_client.wait_for_result()

if not len(sys.argv) == 2 or not (sys.argv[1] == 'pull' or sys.argv[1] == 'push'):
    print 'usage: rosrun jeff_moveit test_door.py <pull/push>'
    sys.exit(0)

rospy.init_node('test_move')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.sleep(2.0)

# computer vision svcs
segment_objects = rospy.ServiceProxy('segment_objects', SegmentObjects)
yolo = rospy.ServiceProxy('/yolo_detection', YoloDetection)

# moveit svcs
ps_ctl = rospy.ServiceProxy('planning_scene_ctl', PlanningSceneControl)
add_box = rospy.ServiceProxy('add_collision_box', AddCollisionBox)
add_grid = rospy.ServiceProxy('add_occupancy_grid', AddOccupancyGrid)
remove_box = rospy.ServiceProxy('remove_box', RemoveBox)
pick = rospy.ServiceProxy('pick_object', Pick)
move = rospy.ServiceProxy('move_gripper', Move)


ps_ctl('CLEAR_ATTACH')
ps_ctl('CLEAR_COLLISION')
play_motion('look_default')


# get pcl and transform to base_footprint
pclmsg = get_pclmsg(10)
pclmsg = transform_pclmsg('base_footprint', pclmsg)

# get x,y,z and r,g,b images
pclxyz = pclxyz_from_pclmsg(pclmsg)
frame, img = frame_imgmsg_from_pclmsg(pclmsg)



# some great magic number thresholding
selection = None
detections = yolo(img, 'door_and_handle_custom', 0.2, 0.3).detected_objects

for detection in detections:
    x,y,w,h = [int(i) for i in detection.xywh]
    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
    cv2.putText(frame, detection.name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    pclx_boxed = pclxyz[y:y+h, x:x+w, 0]
    pclx_boxed_sorted = np.sort(pclx_boxed, axis=None)
    percentile_20 = pclx_boxed_sorted[int((h*w - 1) * 0.2)]

    min3d = pclxyz[y,x,:]
    max3d = pclxyz[y,x,:]
    
    for i in range(h):
        for j in range(w):
            if pclx_boxed[i,j] < percentile_20:
                frame[y+i, x+j] = [0,255,0]
                point3d = pclxyz[y+i, x+j, :]

                min3d = np.minimum(min3d, point3d)
                max3d = np.maximum(max3d, point3d)
    
    size = max3d - min3d
    size = Vector3(*size)
    centre_point = (max3d + min3d)/2.0
    centre_point = Point(*centre_point)
    add_box('/base_footprint', 'grab_this', size, centre_point, Quaternion(*quaternion_from_euler(0,0,0)))

    # motions = ['look_left', 'look_right', 'look_straight']
    motions = ['look_straight']

    for i, motion in enumerate(motions):
        play_motion(motion)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        # remove 8cm around the item we wana pick up - for occupancy grid
        pcl = remove_box(pcl, Point(*min3d), Point(*max3d), 0.08)

        # pass this pcl to the grid creator
        max_p = Point(2,2,1)
        min_p = Point(-1,-1,0.5)
        res = add_grid('base_footprint', 'grid' + str(i), Vector3(0.05,0.05,0.05), Vector3(0.05,0.05,0.05), min_p, max_p, pcl.points)


    # pre-approach
    # pose = PoseStamped()
    # pose.header.frame_id = 'base_footprint'
    # pose.header.stamp = rospy.Time.now()
    # pose.pose.position.x = 0.3
    # pose.pose.position.y = -0.3
    # pose.pose.position.z = centre_point.z
    # pose.pose.orientation = Quaternion(*quaternion_from_euler(3.142,0,0))
    # res = move(pose)
    # print res.pose.pose

    # grab and activate the handle
    position = PointStamped(pclmsg.header, centre_point)
    position.point.y -= size.y/2.0
    selection = 'grab_this', position, size, Vector3(0,0,-0.15)
    res = pick(*selection)
    print res.pose.pose

    if sys.argv[1] == 'pull':
        # open the door just a little
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = res.pose.pose.position.x - 0.2
        pose.pose.position.y = res.pose.pose.position.y
        pose.pose.position.z = centre_point.z
        pose.pose.orientation = res.pose.pose.orientation
        res = move(pose)
        print res.pose.pose

        # reverse
        cmd_vel(-0.3)

        # reset
        play_motion('open_gripper')
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0.3
        pose.pose.position.y = -0.4
        pose.pose.position.z = centre_point.z
        pose.pose.orientation = res.pose.pose.orientation
        move(pose)
        print res.pose.pose

        pcl = get_pclmsg()
        max_p = Point(2,2,1)
        min_p = Point(-1,-1,0.5)
        add_grid('base_footprint', 'grid' + str(i), Vector3(0.05,0.05,0.05), Vector3(0.05,0.05,0.05), min_p, max_p, pcl)

        # behind door
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = res.pose.pose.position.x + 0.2
        pose.pose.position.y = res.pose.pose.position.y + 0.3
        pose.pose.position.z = centre_point.z
        pose.pose.orientation = res.pose.pose.orientation
        res = move(pose)
        print res.pose.pose

        ps_ctl('CLEAR_COLLISION')

        # open door
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0.5
        pose.pose.position.y = -0.5
        pose.pose.position.z = centre_point.z
        pose.pose.orientation = res.pose.pose.orientation
        res = move(pose)
        print res.pose.pose



    
    elif sys.argv[1] == 'push':
        ps_ctl('CLEAR_COLLISION')

        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = res.pose.pose.position.x + 0.35
        pose.pose.position.y = res.pose.pose.position.y + 0.35
        pose.pose.position.z = centre_point.z
        pose.pose.orientation = res.pose.pose.orientation
        res = move(pose)
        print res.pose.pose

        cmd_vel(1)

    break



# print ps_ctl('MOVE')
cv2.imshow('test', frame)
cv2.waitKey(0)
# rospy.sleep(5)
print ps_ctl('CLEAR_ATTACH')
print ps_ctl('CLEAR_COLLISION')
print ps_ctl('Cbelheh')

# add_box(frame_id, obj_id, dims, p, q)
