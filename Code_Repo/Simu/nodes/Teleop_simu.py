#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import UInt16MultiArray,Float32MultiArray
import sys, termios, tty
import click
from geometry_msgs.msg import Twist
import select



class get_data:
	def __init__(self):
		print("initalizing publisher and subscriber")
		self.twist = Twist() #object of the type that we will publish
		self.im=[] #array that will contain the cam data
		self.laz=[] #array that will contain the scan data

		

	def callbackIm(self,msg): #callback to get image_data
		self.im=msg.data
		
	def callbackLaz(self,msg): #callback to get laser_data
		self.laz=msg.data



	def main(self):
		rospy.init_node('Teleop')
		self.subIm = rospy.Subscriber("/cam_data", UInt16MultiArray, self.callbackIm)
		self.sublaz = rospy.Subscriber("/scan_data", Float32MultiArray, self.callbackLaz)
		self.paramvel_ang=rospy.get_param("/angular_scale", 9) #parameter for angular speed with a default value
		self.paramvel_lin=rospy.get_param("/linear_scale",0.1)#parameter for linear speed with a default value
		self.channel_vel = rospy.get_param("/vel_channel", '/cmd_vel')
		self.pub = rospy.Publisher(self.channel_vel, Twist, queue_size=1)
		self.rate = rospy.Rate(20)
		
		while not rospy.is_shutdown():
			if(self.im !=[] and self.laz !=[]): #if we have both data types
			
				lines_mean =self.im[1] # our lines center
				center_img = self.im[2] #our image center
		
				nb_Laz=self.laz[1] # the number that will determine if we are in lidar navigation mode or not
				dist_left=self.laz[2] #distance to obstacles
				dist_right=self.laz[3]
				nb_avoidance=self.laz[4] #1 when we need to avoid object and 0 otherwise
				nb_line =self.laz[5]# 1 when we don't need line following
				

				
				if dist_left != 0:

					ajj = -2.2*(1-((dist_left-0.17)*4)) #the adjustment we will make to the angle when an obstacle is found 
					param=0.3 #the param will determine the weight of line following over obstacle avoidance
					
				if dist_right !=0:

					ajj = +2.2*(1-((dist_right-0.17)*4))
					param=0.3
					
				if ((dist_left ==0) and (dist_right ==0)): #case where there is no obstacle
					ajj = 0
					if (nb_Laz!=1):
						param =1.4
			
					else: 
						param =0.3

				print(param)

				vit_laz =1.3*self.paramvel_lin*(1-np.abs((self.laz[0]))) # linear speed in lidar mode
				ang_laz = +0.3*self.paramvel_ang*(self.laz[0])#angular speed in lidar mode
				ang_im= param*self.paramvel_ang*(center_img-lines_mean)/(center_img) #angular speed in line following mode
				vit_im =np.abs(2.2*self.paramvel_lin*(1-np.abs((center_img-lines_mean)/(center_img-70)))) #linear speed in line following mode
				self.twist.linear.x =vit_laz*nb_Laz+vit_im*0.7 #the final speeds with every contributions
				self.twist.angular.z = ang_laz*nb_Laz + ang_im*(1-nb_line) +ajj*nb_avoidance*1
				self.pub.publish(self.twist)
				self.rate.sleep() #to set the publishing rate to our rate that we chose earlier
		
		rospy.spin()

if __name__ == '__main__':
	test = get_data()
	test.main()
	


