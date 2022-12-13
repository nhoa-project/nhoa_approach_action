import rospy
import actionlib
from nhoa_approach_action.msg import *
from move_base_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node("test_approach_action")

    ac = actionlib.SimpleActionClient("Approach", ApproachAction)
    #mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # counter = 0
    # while not mb.wait_for_server(rospy.Duration(1.0)) and counter < 20:
    #     rospy.logwarn("Waiting for move_base actionlib server to start...")
    #     rospy.sleep(1.0)
    #     counter+=1

    counter = 0    
    while not ac.wait_for_server(rospy.Duration(1.0)) and counter < 20:
        rospy.logwarn("Waiting for Approach actionlib server to start...")
        rospy.sleep(1.0)
        counter+=1
        
    rospy.loginfo("Action Approach ready to execute.")
    rospy.sleep(5.0)
    rospy.loginfo("Sending Approach goal")
    agoal =ApproachGoal()
    agoal.target_id = "-1"
    ac.send_goal(agoal)
    rospy.loginfo("Approach goal sent!")
    rospy.loginfo("Executing for 40 seconds")
    rospy.sleep(60.0)
    rospy.loginfo("Cancelling Approach action and exiting")
    ac.cancel_all_goals()
    