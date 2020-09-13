import rospy
import numpy as np
import laser_geometry as lg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, Point
from jeff_moveit.srv import PlanningSceneControl, AddOccupancyGrid
import time

rospy.init_node('test_laser_occupancy')
lp = lg.LaserProjection()
ps_ctl = rospy.ServiceProxy('planning_scene_ctl', PlanningSceneControl)
add_grid = rospy.ServiceProxy('add_occupancy_grid', AddOccupancyGrid)

# clear prev collisions for our test
ps_ctl('CLEAR_COLLISION')
start_time = time.time()

# project laserscan to pcl
scan = rospy.wait_for_message('/scan', LaserScan)
pcl = lp.projectLaser(scan)

# laser is on the floor, push the centre point up for our wall
start_offset = time.time()
cloud = np.fromstring(pcl.data, dtype=np.float32)
cloud = cloud.reshape(pcl.height, pcl.width, int(pcl.point_step/4))
cloud[:,:,2] += 1
pcl.data = list(cloud.view(np.uint8).flatten())
end_offset = time.time()

# create wall from points
leaf_size = Vector3(0.01,0.01,0.01)
box_size = Vector3(0.02,0.02,2.5)
max_p = Point(2,2,2)
min_p = Point(-1,-2,0)
res = add_grid('base_footprint', 'grid', leaf_size, box_size, min_p, max_p, pcl)

end_time = time.time()
print 'time to offset pcl', end_offset - start_offset
print 'time to do everything', end_time - start_time