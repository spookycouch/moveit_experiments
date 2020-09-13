import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

def move(x_dist, speed=0.5):
    max_vel = rospy.get_param('/mobile_base_controller/linear/x/max_velocity')
    min_vel = rospy.get_param('/mobile_base_controller/linear/x/min_velocity')

    if x_dist < 0:
        speed = -speed
        speed = max(speed, min_vel)
    else:
        speed = min(speed, max_vel)
    
    time_req = rospy.Duration(x_dist/float(speed))
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)

    twist_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = speed
    
    while rospy.Time.now() - start_time < time_req:
        twist_pub.publish(twist)
        rate.sleep()

def move_to_base_footprint(x,y):
    pass
    # TODO:
    # apparently physics stackex says this is fine
    # https://physics.stackexchange.com/questions/184347/a-car-with-constant-speed-doing-a-turn
    #
    # get eqn of circle for (0,0) (x,y) (2x,0)
    # get angle between (0,0) and (x,y)
    # 
    # get circumference between (0,0) and (x,y)
    # get time to traverse circle given speed
    # 
    # find angle we need to turn p/s 

if __name__ == '__main__':
    rospy.init_node('move_the_boy')
    rospy.wait_for_message('/clock', Clock)
    move(-2)