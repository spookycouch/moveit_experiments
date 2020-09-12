#! /usr/bin/env python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def play_motion(motion_name):
    # Wait for the play motion server to come up and send goal
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(1.0))

    # Create the play_motion goal and send it
    pose_goal = PlayMotionGoal()
    pose_goal.motion_name = motion_name
    pose_goal.skip_planning = True
    play_motion_client.send_goal(pose_goal)
    try:
        play_motion_client.wait_for_result(1.0)
    except:
        pass

if __name__ == '__main__':
    rospy.init_node('quick_play_motion')
    while not rospy.is_shutdown():
        play_motion('home')