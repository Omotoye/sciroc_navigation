#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the go_to_poi action
from pal_navigation_msgs.msg import GoToPOIAction, GoToPOIGoal

# for the service messages used 
from sciroc_navigation.srv import GoToPOI, GoToPOIResponse


def go_to_poi(poi_req):
    """This function takes in the a poi_req argument which represents the point
    of interest in the simulation which the robot is required to navigate to. It 
    then sends that string to the go_to_poi action server to reach that goal. 

    Args:
        poi_req (string): The ID of a point of interest in the simulation environment  

    Returns:
        [string]: result sent from the go_to_poi action server. 
    """
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

    # return the result of executing the action
    return client.get_result()  


def handle_go_to_poi(req):
    """This is a callback function for handling the request from the go_to_poi_service 

    Args:
        req (sciroc_navigation.srv.GoToPOIRequest): The request message sent from the 
        client

    Returns:
        [sciroc_navigation.srv.GoToPOIResponse]: The response message sent back to the 
        client
    """
    
    # Checking if the Point of Interest exist
    if (rospy.has_param('/mmap/poi/submap_0/{goal}'.format(goal=req.goal))):
        go_to_poi(req.goal)
        return GoToPOIResponse('goal reached')
    else:
        return GoToPOIResponse('Invalid POI')


def main():
    rospy.init_node('go_to_poi_')
    
    # Initialize the service 
    s = rospy.Service('go_to_poi_service', GoToPOI, handle_go_to_poi)
    
    # Keeps the node alive to listen for client requests
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 