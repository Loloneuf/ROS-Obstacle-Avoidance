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
	
		dist_max=0.30 #  the maximal distance that we take into account 
		self.arr.data=msg.ranges #getting the data
		dist_left=np.array(self.arr.data[20:80]) #the upper left part of lidar data
		dist_right=np.array(self.arr.data[280:340]) # the upper right part of lidar datas
		nb_point_left=len(np.extract(dist_left!=np.inf,dist_left)) # number of non infinite distances in dist_left
		nb_point_right=len(np.extract(dist_right!=np.inf,dist_right))
		
		if ((nb_point_left+nb_point_right)>100): #if the number of points detected is above that threshold
			nb_fin =1 # 1 will activate the lidar naviguation mode 
		else :
			nb_fin =0
			
		#print(nb_point_left+nb_point_right)
		
		
		dist_left =np.where(dist_left>dist_max,dist_max,dist_left) #if values are superior to the max, set them to the max
		dist_right =np.where(dist_right>dist_max,dist_max,dist_right) 
		dist_left =np.where(dist_left==np.inf ,dist_max,dist_left) #if values are inf, set them to the max 
		dist_right= np.where(dist_right==np.inf ,dist_max,dist_right)  
		range_left=np.mean(dist_left) # compute the mean of distances
		range_right=np.mean(dist_right)
		#print(nb_fin)
		self.dat.data=[(range_left-range_right)/0.25,nb_fin]# divide by 0.25 to normalize our data
		self.pub.publish(self.dat)

		
		
		
	def main(self):
		rospy.init_node("scan_get_data", anonymous=True)
		self.rate = rospy.Rate(10)
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.pub = rospy.Publisher("/scan_data", Float32MultiArray, queue_size=1) 
		rospy.spin()




if __name__ == '__main__':
	test = get_data()
	test.main()




