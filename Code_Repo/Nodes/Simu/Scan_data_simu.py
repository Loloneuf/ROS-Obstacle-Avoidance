#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray
from image_tools import ImageTools
import cv_bridge
import cv2





class get_data:
	def __init__(self):
		print("initalizing publisher and subscriber")
		self.arr=UInt16MultiArray() # used to stock the datathat we receive 
		self.dat=Float32MultiArray() #used to stock the data that we will publish

	
	def callback(self,msg):
		self.arr.data=msg.ranges #getting the data

		
		
		
	def main(self):
		rospy.init_node("cam_test", anonymous=True)
		self.rate = rospy.Rate(20)
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.pub = rospy.Publisher("/scan_data", Float32MultiArray, queue_size=1) 
		
		dist_max=0.35 #  the maximal distance that we take into account 
		
		while not rospy.is_shutdown():
			if self.arr.data !=[]: #if we have both data types
					
				
				dist_left_corr=np.array(self.arr.data[10:80])#the upper left part of lidar data used for corridor
				dist_right_corr=np.array(self.arr.data[279:349])#the upper right part of lidar data used for corridor
				
				left_side=np.array(self.arr.data[80:100]) #only the points on the side of the robots, used for corridor detection
				right_side=np.array(self.arr.data[260:280])
				
				front_left=np.array(self.arr.data[0:30])  #only the points in front of the robots, used for corridor detection
				front_right=np.array(self.arr.data[329:359])
				
				
				dist_obst_left=np.array(self.arr.data[10:65]) #the points used for obstacle avoidance
				dist_obst_right=np.array(self.arr.data[294:349])
				
				
				nb_points_right_side=len(np.extract(right_side!=np.inf,right_side))#the number of points detected on the side
				nb_points_left_side=len(np.extract(left_side!=np.inf,left_side))
				
				nb_points_front_right=len(np.extract(front_right!=np.inf,front_right)) # the number of points in front of the robot
				nb_points_front_left=len(np.extract(front_left!=np.inf,front_left))
				
				
				dist_obst_left=np.extract(dist_obst_left<0.25,dist_obst_left) #distances to obstacles
				dist_obst_right=np.extract(dist_obst_right<0.25,dist_obst_right)
				
				if (dist_obst_left.size >0):  # computation of the mean distances to obtacles
					nb_points_obst_left=np.mean(dist_obst_left)
				else: 
					nb_points_obst_left=0
				
				if (dist_obst_right.size >0):
					nb_points_obst_right=np.mean(dist_obst_right)
				else: 
					nb_points_obst_right =0
				
				
				if (((nb_points_right_side+nb_points_left_side)>38) or ((nb_points_front_left+nb_points_front_right)==60)): 
					nb_corridor =1  # if there's a sufficient amount of points on sides or in front of the robot, activate corridor mode
				else :
					nb_corridor =0
					
				if ((nb_points_right_side+nb_points_left_side)>38):
				
					nb_corridor_is_active =1 #will allow us to deactivate line following in the corridor
				else:
					nb_corridor_is_active =0
				
				if ((nb_corridor==1)):  # to deactivate obstacle avoidance when in corridor mode
					nb_avoidance =0
				else:
					nb_avoidance=1
					
				
				#print(nb_points_front_left+nb_points_front_right)
				dist_left_corr =np.where(dist_left_corr>dist_max,dist_max,dist_left_corr) #if values are superior to the max, set them to the max
				dist_right_corr =np.where(dist_right_corr>dist_max,dist_max,dist_right_corr)
				dist_left_corr =np.where(dist_left_corr==np.inf ,dist_max,dist_left_corr) #if values are infinite, set them to the max 
				dist_right_corr= np.where(dist_right_corr==np.inf ,dist_max,dist_right_corr)  
				range_left=np.mean(dist_left_corr)# mean of the distances
				range_right=np.mean(dist_right_corr)
				#print(nb_fin)
				self.dat.data=[(range_left-range_right)/0.25,nb_corridor,nb_points_obst_left,nb_points_obst_right,nb_avoidance,nb_corridor_is_active] #what we publish
				self.pub.publish(self.dat)
				self.rate.sleep() #to set the publishing rate to our rate that we chose earlier

					
			
		rospy.spin()




if __name__ == '__main__':
	test = get_data()
	test.main()



