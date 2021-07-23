#! /usr/bin/env python

import rospy
from sciroc_navigation.srv import GoToPOI 


def call_POI_service():
	rospy.wait_for_service('go_to_poi_service')
	try:
		poi_ = raw_input("Enter a Valid Point of Interest: ")
		go_to_poi = rospy.ServiceProxy('go_to_poi_service', GoToPOI)
		result = go_to_poi(poi_)

		if (result.result == 'goal reached'):
			print result.result 
		else: 
			print 'Point of interest [{poi}] does not exist'.format(poi=poi_)
	except rospy.ServiceException as e:
		print 'Service call failed: {e}'.format(e=e)

def main():
	rospy.init_node('dummy_client')

	call_POI_service()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass 

		