#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as n
import sys
import math
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
roslib.load_manifest('lab4')

#Global parameter
land_marks = [[125,525],[125,325],[125,125],[425,125],[425,325],[425,525]]
#Define threshold
threshold = 0.1
#Define grid 7m*7m coverage
grid_coverage = 700
#Define cell size
cell_coverage = 20
#The discretization size as 360
#Define total grids: x = 700/20; y = 700/20; z = 360 degree / 10 degree
grids = n.zeros((35,35,36))
#Define a temp grid
temp_grid = n.zeros((35,35,36))

def compute_location(rot1, rot2,trans):
    x = rot1 * 20 + 10
    y = rot2 * 20 + 10
    rot = -180 + trans * 90 + 45
    return rot, x, y

def compute_trans(x1,x2,y1,y2):
    trans = n.sqrt((x1 - x2)**2 +(y1 - y2)**2)
    return trans

#Compute the angle related the xy coordinate
def compute_angle(x1,x2,y1,y2):
    angle = n.degrees(n.arctan2(y1 - y2,x1 - x2))
    return angle
#Adjus the angle
def adjust(angle):
    if angle > 180:
        angle - = 180
    elif angle <-180:
        angle += 360
    return angle

def compute_robot_motion(robot_rot_1,robot_rot_2,robot_trans,trav_rot_1,trav_rot_2,trav__trans):
    #Compute the robot's location and heading angle data
    r1, x1, y1 = compute_location(robot_rot_1,robot_rot_2,robot_trans)
    #Compute the goal's location and heading angle data
    r2, x2, y2 = compute_location(trav_rot_1,trav_rot_2,trav__trans)
    #Compute the transition (distence)
    trans = compute_trans(x1,x2,y1,y2)
    #Compute the angle related to the x,y coordinate
    angle = compute_angle(x1,x2,y1,y2)
    #Compute the rotation ralated to the angle, and just the data into normalized one
    rotation_1 = adjust(r1 - angle)
    rotation_2 = adjust(angle - r1)

    return rotation_1, trans, rotation_2


#Guassian distribution  
def normpdf(x, mean, var):
    a = (2*n.pi*var)**.5
    b = math.exp(-(float(x)-float(mean))**2/(2*var))
    return a/b

def compute_current_node_probility(trav_rot_1,trav_rot_2,trav__trans,rotation_1_normpdf,transition_normpdf,rotation_2_normpdf):
    global temp_grid
    current_node_probility = temp_grid[trav_rot_1,trav_rot_2,trav__trans]*rotation_1_normpdf*transition_normpdf*rotation_2_normpdf
    return current_node_probility

def compute_current_node_measure_probility(trav_rot_1,trav_rot_2,trav_trans,r_pro,t_pro):
    global temp_grid
    current_node_measure_probility = temp_grid[trav_rot_1,trav_rot_2,trav_trans]* r_pro*t_pro
    return current_node_measure_probility

def compute_coordinate(grids,all_prob):
    grids /= all_prob
    max_index = n.argmax(grids)
    y = (max_index / grids.shape[2]) % grids.shape[1]
    x = (max_index /(grids.shape[2] * grids.shape[1])) % grids.shape[0]
    return x,y

def compute_measurement(trav_rot_1,trav_rot_2,trav_trans,tagnum,rot,trans):
    global land_marks
    r,x,y = compute_location(trav_rot_1,trav_rot_2,trav_trans)
    tr = compute_trans(x,land_marks[tagnum,0],y,land_marks[tagnum,1])
    agnle = compute_angle(x,land_marks[tagnum,0],y,land_marks[tagnum,1])
    r1 = adjust(angle - r)
    rot_pro =normpdf(r1,rot,45)
    trans_pro = normpdf(tr,trans,45)
    return rot_pro,trans_pro 


def updata_measure(tn,trans,rot):
    global threshold, grids,temp_grid
    temp_grid = grids
    all_prob = 0
    for trav_rot_1 in range(35):
        for trav_rot_2 in range(35):
            for trav_trans in range(4):
                r_pro,t_pro = compute_measurement(trav_rot_1,trav_rot_2,trav_trans,tagnum,rot,trans)
                current_node_measure_probility = compute_current_node_measure_probility(trav_rot_1,trav_rot_2,trav_trans,r_pro,t_pro)
                grids[trav_rot_1,trav_rot_2,trav_trans] += current_node_measure_probility
                all_prob += current_node_measure_probility
    x,y = compute_coordinate(grids,all_prob)

def do_action(rot1,trans,rot2):
    global grids, threshold, grids, temp_grid
    #At this time "grids" has some position that equals to 1, define the temp grid to do judge, does not afftect
    #grids' data. Finally, update the data in "grids"
    temp_grid = grids
    #Define a veriable that count all the probability 
    all_prob = 0
    #Traverasl all possible position that the robot at in the future.
    for robot_rot_1 in range(35):
        for robot_rot_2 in range(35):
            #Choose 90->360/90 = 4
            for robot_trans in range(4):
                #Avoid the position that robot have taken
                if temp_grid[robot_pos_1,robot_pos_2,robot_trans]<threshold:
                    #traversal all the possible goal location, get the instruction to that goal\
                    for trav_rot_1 in range(35):
                        for trav_rot_2 in range(35):
                            for trav__trans in range(4):
                                rotation_1, transition, rotation_2 = compute_robot_motion(robot_rot_1,robot_rot_2,robot_trans,trav_rot_1,trav_rot_2,trav__trans)
                                
                                rotation_1_normpdf = normpdf(rotation_1, rot1, 45)
                                transition_normpdf = normpdf(transition, trans, 10)
                                rotation_2_normpdf = normpdf(rotation_2, rot2, 45)

                                current_node_probility = compute_current_node_probility(trav_rot_1,trav_rot_2,trav__trans,rotation_1_normpdf,transition_normpdf,rotation_2_normpdf)
                                grids[trav_rot_1,trav_rot_2,trav__trans] += current_node_probility
                                all_prob += current_node_probility
    x,y = compute_coordinate(grids,all_prob)
    f.write(x)
 


def start():
    global land_marks,grids
    rate = rospy.Rate(10)
    #grids[12,28,3] = 1;# Set this position as the start point of robot
    try:
	f = open("/home/first/catkin_ws/src/lab4/script/trajectory.txt","a")
        bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag')# Read bag file
        for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
            if topic == 'Movements':
# if read topic is movement: then get the current rotation and translation data from msg
                rot1 = msg.rotation1
                rot2 = msg.rotation2
                trans = msg.translation * 100
		f.write("Movements####################################\n")
# translate rotation and translaion to euler number
                rot_1 = n.degrees((euler_from_quaternion([rot1.x,rot1.y,rot1.z, rot1.w]))[2])
                rot_2 = n.degrees((euler_from_quaternion([rot2.x,rot2.y,rot2.z, rot2.w]))[2])

                do_action(rot_1,trans,rot_2)

            else:
		f.write("Measurements\n")
# if read topic is measurement. Since the robot have do action, then it needs to update measurement
                ranges = msg.range * 100
                bearings = msg.bearing
                rot3 = n.degrees((euler_from_quaternion(bearings.x,bearings.y,bearings.z,bearings.w))[2])
                update_measure(msg.tagNum, ranges,rot3)

if __name__ == '__main__':
    try:
        rospy.init_node('lab4')
        start()
    except rospy.ROSInterruptException:
        pass    
















































