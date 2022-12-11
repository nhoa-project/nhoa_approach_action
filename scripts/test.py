import rospy
import actionlib
from nhoa_approach_action.msg import *


if __name__ == '__main__':
    rospy.init_node("test_approach_action")

    ac = actionlib.SimpleActionClient("Approach", ApproachAction)

    counter = 0
    while not ac.wait_for_server(rospy.Duration(1.0)) and counter < 20:
        rospy.logwarn("Action state '%s' waiting for Approach actionlib server to start...")
        rospy.sleep(1.0)
        counter+=1
        
    rospy.loginfo("Action Approach ready to execute.")
    rospy.sleep(5.0)
    rospy.loginfo("Sending Approach goal")
    agoal =ApproachGoal()
    agoal.target_id = "actor_103"
    ac.send_goal(agoal)
    rospy.loginfo("Approach goal sent!")
    rospy.loginfo("Executing for 40 seconds")
    rospy.sleep(40.0)
    rospy.loginfo("Cancelling Approach action and exiting")
    ac.cancel_all_goals()
    