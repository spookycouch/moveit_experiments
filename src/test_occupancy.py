import rospy
from jeff_moveit.srv import AddOccupancyGrid, PlanningSceneControl
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point


rospy.init_node('test_occ')
rospy.wait_for_service('add_occupancy_grid')
add_grid = rospy.ServiceProxy('add_occupancy_grid', AddOccupancyGrid)
ps_ctl = rospy.ServiceProxy('planning_scene_ctl', PlanningSceneControl)

pub = rospy.Publisher('occ_grid', PointCloud2, queue_size=10, latch=True)
print ps_ctl('CLEAR')

cloud_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
max_p = Point(2,2,1)
min_p = Point(-1,-1,0.5)
res = add_grid('base_footprint', 'grid', 0.03, 0.01, min_p, max_p, cloud_msg)
# res.points.header.frame_id = 'base_footprint'
print len(res.points.data)
pub.publish(res.points)

print ps_ctl('MOVE')

rospy.spin()