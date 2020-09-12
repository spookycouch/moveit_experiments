#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import PointCloud2, Image
from jeff_segment_objects.srv import SegmentObjects
from lasr_object_detection_yolo.srv import YoloDetection
from cv_bridge import CvBridge, CvBridgeError

import rospy
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

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



def detect_objects(image_raw, dataset, confidence, nms):
    # wait for the service to come up
    rospy.wait_for_service('/yolo_detection')
    # call the service
    try:
        yolo = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        return yolo(image_raw, dataset, confidence, nms)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


def callback(cloud_msg, image_msg):
    print 'callback!'
    rospy.wait_for_service('segment_objects')
    try:
        segment_objects = rospy.ServiceProxy('segment_objects', SegmentObjects)
        # res = segment_objects(cloud_msg, 0.2, 0.035) # coffee
        res = segment_objects(cloud_msg, 0.5, 0.035) # door
        


        width = cloud_msg.width
        height = cloud_msg.height
        cluster_boxes = []

        # for each cluster
        for cluster in res.clusters:
            max_index = cluster.indices[0]
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

            cluster_boxes.append((left, top, right - left, bottom - top)) # xywh
        # print cluster_boxes

        
        objects = detect_objects(image_msg, 'door_and_handle_custom', 0.2, 0.3).detected_objects
        # objects = detect_objects(image_msg, 'costa', 0.5, 0.3).detected_objects
        cost_matrix = np.zeros((len(objects), len(cluster_boxes)))
        for i, obj in enumerate(objects):
            for j, box in enumerate(cluster_boxes):
                cost_matrix[i,j] = calcBoundingBoxIOU(obj.xywh, box)
        cost_matrix = -cost_matrix
        
        matches = linear_sum_assignment(cost_matrix)
        # print matches
        # print cost_matrix



        # get the cv2 image
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError:
            return
        # draw
        # for plane in res.planes:
        #     plane_colour = np.random.randint(0,256,(3))
        #     for index in plane.indices:
        #         row = int(index/width)
        #         col = index%width        
        #         frame[row,col] = plane_colour
        
        for i, cluster in enumerate(res.clusters):
            if (cluster.size.x < 0.01 or cluster.size.y < 0.01) or cluster.size.x > 0.5 or cluster.size.y > 0.5:
                continue

            max_index = cluster.indices[0]
            left = cluster.indices[0]%width
            right = cluster.indices[0]%width
            bottom = int(cluster.indices[0]/width)
            top = int(cluster.indices[0]/width)
            
            obj = None
            colour = (0,0,255)

            for match_index in range(len(matches[0])):
                match = matches[0][match_index], matches[1][match_index]

                if match[1] == i:
                    colour = (0,255,0)
                    obj = objects[match[0]]

            for index in cluster.indices:
                row = int(index/width)
                col = index%width
                left = min(left, col)
                right = max(right, col)
                top = min(top, row)
                bottom = max(bottom, row)
                
                frame[row,col] = colour
            
            if obj is None:
                cv2.putText(frame, 'garbage', (left, top-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            else:
                cv2.putText(frame, obj.name, (left, top-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

            cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 1)
            cv2.putText(frame, 'x:{:.2f}, y:{:.2f}, z:{:.2f}'.format(cluster.size.x, cluster.size.y, cluster.size.z), (left, top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)




        cv2.imshow('image', frame)
        cv2.waitKey(1)
    




    except rospy.ServiceException as e:
        print e

rospy.init_node('get_segmented_objects')
cloud_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2)
image_sub = message_filters.Subscriber('/xtion/rgb/image_rect_color', Image)
ts = message_filters.ApproximateTimeSynchronizer([cloud_sub, image_sub], 1, 0.1)
ts.registerCallback(callback)
rospy.spin()