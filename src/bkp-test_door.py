#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import linear_sum_assignment
from tf.transformations import quaternion_from_euler

from jeff_segment_objects.srv import SegmentObjects, RemoveBox
from jeff_moveit.srv import AddCollisionBox, PlanningSceneControl, AddOccupancyGrid, Pick
from lasr_object_detection_yolo.srv import YoloDetection

from geometry_msgs.msg import Point, Quaternion, PointStamped, Vector3
from sensor_msgs.msg import PointCloud2, Image
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


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
    frame = frame.reshape(480,640,32)
    frame = frame[:,:,16:19]
    imgmsg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    return frame.copy(), imgmsg


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



# Intersection over Union of two bounding boxes
def calcBoundingBoxIOU(a, b):
    # get intersection coords
    xleft = max(a[0], b[0])
    xright = min(a[0] + a[2], b[0] + b[2])
    ytop = max(a[1], b[1])
    ybottom = min(a[1] + a[3], b[1] + b[3])
    # calculate intersection area
    width = max(0, xright - xleft)
    height = max(0, ybottom - ytop)
    intersection = float(width * height)
    # calculate union area
    union = float((a[2] * a[3]) + (b[2] * b[3]) - intersection)
    # return IOU
    if union == 0:
        return 0
    return intersection/union




rospy.init_node('test_move')

# computer vision svcs
segment_objects = rospy.ServiceProxy('segment_objects', SegmentObjects)
yolo = rospy.ServiceProxy('/yolo_detection', YoloDetection)

# moveit svcs
ps_ctl = rospy.ServiceProxy('planning_scene_ctl', PlanningSceneControl)
add_box = rospy.ServiceProxy('add_collision_box', AddCollisionBox)
add_grid = rospy.ServiceProxy('add_occupancy_grid', AddOccupancyGrid)
remove_box = rospy.ServiceProxy('remove_box', RemoveBox)
pick = rospy.ServiceProxy('pick_object', Pick)


ps_ctl('CLEAR_ATTACH')
ps_ctl('CLEAR_COLLISION')
play_motion('look_default')
pcl = get_pclmsg(10)
frame, img = frame_imgmsg_from_pclmsg(pcl)



# ADD MOVEIT COLLISION OBJECTS TO SCENE
# TODO: probably dont need segmentation
res = segment_objects(pcl, 0.4, 0.05) # coffee
height, width, channels = frame.shape
cluster_boxes = []
count = 0

for cluster in res.clusters:
    left = cluster.indices[0]%width
    right = cluster.indices[0]%width
    bottom = int(cluster.indices[0]/width)
    top = int(cluster.indices[0]/width)

    for index in cluster.indices:
        row = int(index/width)
        col = index%width
        left = min(left, col)
        right = max(right, col)
        top = min(top, row)
        bottom = max(bottom, row)
        frame[row, col] = (0,255,0)

    if cluster.size.x * cluster.size.y * cluster.size.z < 0.1:
        add_box('/base_footprint', str(count), cluster.size, cluster.point, Quaternion(*quaternion_from_euler(0,0,0)))
        cluster_boxes.append(((left, top, right - left, bottom - top), str(count), cluster)) # xywh, id
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 1)
    
    count += 1



# IOU MATCHING WITH YOLO
objects = yolo(img, 'door_and_handle_custom', 0.2, 0.3).detected_objects
cost_matrix = np.zeros((len(objects), len(cluster_boxes)))
for i, obj in enumerate(objects):
    for j, (box,id, cluster) in enumerate(cluster_boxes):
        cost_matrix[i,j] = calcBoundingBoxIOU(obj.xywh, box)
cost_matrix = -cost_matrix
matches = linear_sum_assignment(cost_matrix)



selection = None

for i, cluster in enumerate(cluster_boxes):
    for j, index in enumerate(matches[1]):
        if i == index:
            cv2.putText(frame, 'cluster {}: {}'.format(cluster[1], objects[j].name), (cluster[0][0], cluster[0][1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            if objects[j].name == 'door handle':
                selection = cluster

if selection is not None:
    cluster = selection

    min_point = Point()
    max_point = Point()

    min_point.x = cluster[2].point.x - cluster[2].size.x/2.0
    min_point.y = cluster[2].point.y - cluster[2].size.y/2.0
    min_point.z = cluster[2].point.z - cluster[2].size.z/2.0

    max_point.x = cluster[2].point.x + cluster[2].size.x/2.0
    max_point.y = cluster[2].point.y + cluster[2].size.y/2.0
    max_point.z = cluster[2].point.z + cluster[2].size.z/2.0

    motions = ['look_left', 'look_right', 'look_straight']

    for i, motion in enumerate(motions):
        play_motion(motion)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        # remove 8cm around the item we wana pick up - for occupancy grid
        pcl = remove_box(pcl, min_point, max_point, 0.08)

        # pass this pcl to the grid creator
        max_p = Point(2,2,1)
        min_p = Point(-1,-1,0.5)
        res = add_grid('base_footprint', 'grid' + str(i), 0.05, 0.05, min_p, max_p, pcl.points)

    position = PointStamped(cluster[2].header, cluster[2].point)
    size = Vector3(cluster[2].size.x, cluster[2].size.y, min(0.08, cluster[2].size.z))
    print size
    selection = cluster[1], position, Vector3(0,0,0), size

    pick(*selection)



# print ps_ctl('MOVE')
cv2.imshow('test', frame)
cv2.waitKey(0)
# rospy.sleep(5)
print ps_ctl('CLEAR_ATTACH')
print ps_ctl('CLEAR_COLLISION')
print ps_ctl('Cbelheh')

# add_box(frame_id, obj_id, dims, p, q)
