#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

roslib.load_manifest('lab4')

threshold = 0.1

line_marker = Marker()	

total_coverage = 700
total_heading = 90
cell_coverage = 20
cell_heading = 20
initial_pos = [12,28,3]
pos_array = np.zeros(((total_coverage/cell_coverage),(total_coverage/cell_coverage),(total_heading/cell_heading)))

landmark_coords = [[0,0],[125,525],[125,325],[125,125],[425,125],[425,325],[425,525]]

max_iter = 35

def rotation_adjust(rot_z):
	if rot_z < -180:
		rot_z += 360
	elif rot_z > 180:
		rot_z -= 360
	return rot_z

def get_rot_and_pos(rotational_measure,rot_z,translational_measure,trans):
	rotation_pos = 0.0088653 * np.power(2.718, -1.0 * (((rotational_measure - rot_z)**2)/4050.0))
	translational_pos = 0.0398942 * np.power(2.718, -1.0 * (((translational_measure - trans)**2)/200.0))
	return rotation_pos,translational_pos

def get_change(trans1,trans2,trans3,translational_pos,rotation_pos):
	global pos_array
	change = pos_array[trans1, trans2, trans3]
	change *= rotation_pos
	change *= translational_pos
	return change

def find_change(rt1,rt2,rt3,rt4,trans1,trans2):
	global pos_array
	global threshold

	if pos_array[rt2, rt3, rt4] > threshold:
		rt1 += 1
		if(rt1 == 4):
			rt1 = 0
			trans2 += 1
		if(trans2 == max_iter):
			rt1 = 0
			trans2 = 0
			trans1 += 1
		if(trans1 == max_iter):
			rt1 = 0
			trans1 = 0
			trans2 = 0
			rt4 += 1
		if(rt4 == 4):
			rt1 = 0
			rt4 = 0 
			trans1 = 0
			trans2 = 0
			rt3 += 1
		if(rt3 == max_iter):
			rt1 = 0
			rt3 = 0
			rt4 = 0
			trans1 = 0
			trans2 = 0
			rt2 += 1
	else:
		rt4 += 1
		if(rt4 == 4):
			rt1 = 0
			rt4 = 0
			trans1 = 0
			trans2 = 0
			rt3 += 1
		if(rt3 == max_iter):
			rt1 = 0
			rt3 = 0
			rt4 = 0
			trans1 = 0
			trans2 = 0
			rt2 += 1

	return rt1, rt2, rt3, rt4, trans1, trans2


def observation_change(tagnum, trans, rot_z):
	global pos_array
	trans1 = 0
	trans2 = 0
	trans3 = 0
	tp = 0
	while(trans1 < max_iter):
		translational_measure = np.sqrt(((10 + 20*trans1) - landmark_coords[tagnum+1][0]) ** 2 + ((10 + 20*trans2)  - landmark_coords[tagnum+1][1]) ** 2)
		rotational_measure = rotation_adjust(np.degrees(np.arctan2(landmark_coords[tagnum+1][0]-(10 + 20*trans2), landmark_coords[tagnum+1][1] - (10 + 20*trans1))) - (-135 + trans3*90))
		rotation_pos,translational_pos = get_rot_and_pos(rotational_measure,rot_z,translational_measure,trans)
		tp += get_change(trans1, trans2, trans3, translational_pos, rotation_pos)
		change = translational_pos * rotation_pos
		pos_array[trans1, trans2, trans3] *= change
		trans3 += 1
		if(trans3 == 4):
			trans3 = 0
			trans2 += 1
		if(trans2 == max_iter):
			trans3,trans2 = 0,0
			trans1 += 1
		
	pos_array /= tp
	mx = np.argmax(pos_array)
	y = (mx/4) % max_iter
	x = (mx/140) % max_iter
	f = open("trajectory.txt","a")
        f.write("work")
        #f.write(y)
	#drawLine(x, y)


def movement_change(rot_1, trans, rot_2):
	global pos_array, threshold
	trans1 = 0
	trans2 = 0
	tp = 0
	rt1 = 0
	rt2 = 0
	rt3 = 0
	rt4 = 0
	while(rt2 < max_iter):	
		if pos_array[rt2, rt3, rt4] >= threshold:
			trans1x = 20*trans1 + 10
			trans1y = 20*trans2 + 10
			rot_11 = (-180) + (rt1 * 90) + 45
			trans2x = 20*rt2 + 10
			trans2y = 20*rt3 + 10
			rot_21 = (-180) + (rt4 * 90) + 45
			transt = np.sqrt((trans1x - trans2x) ** 2 + (trans1y - trans2y) ** 2)
			rot_1t = np.degrees(np.arctan2(trans1y-trans2y, trans1x - trans2x)) - rot_21
			rot_2t = rot_11 - np.degrees(np.arctan2(trans1y-trans2y, trans1x - trans2x))
			rot_1t = rotation_adjust(rot_1t)
			rot_2t = rotation_adjust(rot_2t)
			rot_1p,transp = get_rot_and_pos(rot_1t,rot_1,transt,trans)
			rot_2p,transp2 = get_rot_and_pos(rot_2t,rot_2,transt,trans)
			incr = get_change(rt2, rt3, rt4, transp, rot_1p)
			pos_array[trans1, trans2, rt1] += incr * rot_2p
			tp += incr * rot_2p
		
		rt1,rt2,rt3,rt4,trans1,trans2 = find_change(rt1,rt2,rt3,rt4,trans1,trans2)


	pos_array /= tp
	mx = np.argmax(pos_array)
	y = (mx/4) % max_iter
	x = (mx/140) % max_iter
	print("move: ",x,y)
	f = open("trajectory.txt","a")
        f.write("work")
        #f.write(y)

def readbagfile():
	try:
		bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag')
		f = open("/home/first/catkin_ws/src/lab4/script/trajectory.txt","a")
        	
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			
			if topic == 'Movements': 
				f.write("Movements\n")
				rot_1 = msg.rotation1
				rot_2 = msg.rotation2
				trans = msg.translation * 100
				rot_1 = np.degrees((euler_from_quaternion([rot_1.x,rot_1.y,rot_1.z,rot_1.w]))[2])
				rot_2 = np.degrees((euler_from_quaternion([rot_2.x,rot_2.y,rot_2.z,rot_2.w]))[2])
				movement_change(rot_1, trans, rot_2)
			
			if topic == 'Observations':
				f.write("Observations\n")
				range_val = msg.range * 100
				bearingval = msg.bearing
				tagnum = msg.tagNum
				rot_z = np.degrees((euler_from_quaternion([bearingval.x, bearingval.y, bearingval.z, bearingval.w]))[2])
				observation_change(tagnum, range_val, rot_z)
		
	finally:
		bag.close()

if __name__ == '__main__':
	rospy.init_node('lab4')
	readbagfile()











































