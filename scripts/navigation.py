#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pal_navigation_msgs.msg import GoToPOIAction, GoToPOIGoal

from sciroc_navigation.srv import GoToPOI 
from sciroc_navigation.srv import GoToPOIResponse


def go_to_poi(poi_req):
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/poi_navigation_server/go_to_poi', GoToPOIAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # creating a new goal with the go to POI constructor
    goal = GoToPOIGoal()
    goal.poi.data = poi_req  

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


def handle_go_to_poi(req):
    if (rospy.has_param('/mmap/poi/submap_0/{goal}'.format(goal=req.goal))):
        go_to_poi(req.goal)
        return GoToPOIResponse('goal reached')
    else:
        return GoToPOIResponse('Invalid POI')


def main():
    """
    Initializes the Service and sends request message to
    the callback function
    """
    rospy.init_node('go_to_poi_')
    s = rospy.Service('go_to_poi_service', GoToPOI, handle_go_to_poi)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 