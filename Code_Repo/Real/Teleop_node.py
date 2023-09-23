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
		self.lock = False #condition for obstacle avoidance 
		

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
				center_yellow=self.im[3] #center of the yellow points for obstacle avoidance 
				
				if(center_yellow <(center_img -10)): #we define a line around the center that will trigger a adjustment if it is trespassed
					print('GAUCHE')
					ajj = +0.5 #adjustment that we will use to correct the robot angle
				elif(center_yellow >(center_img +10  )):
					print('DROITE')
					ajj = -0.5
				else: #if we are already centered, no adjustment requiered
					ajj =0;
					
				

				nb_Laz=self.laz[1] # the number that will determine if we are in lidar navigation mode or not
				vit_laz =0.7*self.paramvel_lin*(1-np.abs((self.laz[0]))) # linear speed in lidar mode
				ang_laz = -0.3*self.paramvel_ang*(self.laz[0])#angular speed in lidar mode
				ang_im= 0.4*self.paramvel_ang*(centerimg-centermean)/(centerimg) #angular speed in line following mode
				vit_im =2.2*self.paramvel_lin*(1-np.abs((centerimg-centermean)/(centerimg-100))) #linear speed in line following mode
				self.twist.linear.x =vit_laz*nb_Laz+(1-nb_Laz)*vit_im #the final speeds with every contributions
				self.twist.angular.z = ang_laz*nb_Laz +(1-nb_Laz)*ang_im +ajj
				self.pub.publish(self.twist)
				self.rate.sleep() #to set the publishing rate to our rate that we chose earlier
		rospy.spin()

if __name__ == '__main__':
	test = get_data()
	test.main()
	


