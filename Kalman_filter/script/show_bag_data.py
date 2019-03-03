#!/usr/bin/env python
import rosbag
import rospy

rospy.init_node('show_bag')
print ('hello WROLD')
bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag','r')
for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
	print topic
	print msg
	print t	
bag.close()
