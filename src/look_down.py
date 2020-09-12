#! /usr/bin/env python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

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

if __name__ == '__main__':
    import yaml
    import rospkg
    PATH = rospkg.RosPack().get_path('jeff_moveit') + '/src/'

    rospy.init_node('quick_play_motion')
    with open(PATH + 'motions.yaml') as f:
        look_down = yaml.safe_load(f)['play_motion']['motions']['look_down']
    
    rospy.set_param('play_motion/motions/look_down', look_down)
    play_motion('look_down')