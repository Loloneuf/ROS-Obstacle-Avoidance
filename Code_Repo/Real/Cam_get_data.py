#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16MultiArray
from image_tools import ImageTools
import cv_bridge
import cv2

yellow_flag = 0
r,l = 1,1
y = [0,315]

class get_data:  # the class format allow us to have a cleaner code structure as well as a an efficient way to save our variables
	def __init__(self):
		print("initalizing publisher and subscriber")
		self.arr=UInt16MultiArray() #the object that wil store what we will publish
		self.cvBridge = cv_bridge.CvBridge() #the object that will store our image
		self.IMG = ImageTools() # an object from a usefull library that makes easier the conversion between types

	def callback(self,msg):
		global r,l,y,yellow_flag # used for obstacle avoidance
		self.arr.data=msg.data #to get the data from the publisher
		cvImage1 = self.IMG.convert_ros_compressed_to_cv2(msg) 
		cvImage = cvImage1[len(cvImage1)//2::,5:len(cvImage1[0])-5] # crop to only get the low part of the camera
		hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV) #convert our image in a HSV format

			
	
		#red filter
		low_r =np.array([0,40,50])
		up_r =np.array([20,250,255])
		
		
		#blue filter
		low_b =np.array([95,40,100])
		up_b =np.array([140,250,255])    
		
		#green filter
		low_g =np.array([40,40,100])
		up_g =np.array([90,250,255])
		
		
		#yellow filter
		low_y =np.array([22,50,100])
		up_y =np.array([35,255,255])
		
		
		mask_r = cv2.inRange(hsv,(low_r),(up_r))# create a mask to use several operator on it 		
		M = cv2.moments(mask_r) #get the moment to compute the mean of our lines
		lines_mean_x = 0 #variable that will be use to store the x mean between our colored lines
		lines_mean_y = 0

		if M["m00"]>0:
			cX_r = int (M["m10"] / M["m00" ] )
			cY_r = int (M["m01"] / M["m00" ] )
			l = 1
		else:  # case where the red line is not detected
			cX_r = len(cvImage[0])
			cY_r = len(cvImage)//2
			l = 0
			
		mask_g = cv2.inRange(hsv,(low_g),(up_g))
		M = cv2.moments(mask_g)

		if M["m00"]>0:
			 cX_g = int (M["m10"] / M["m00" ] )
			 cY_g = int (M["m01"] / M["m00" ] )
			 r = 1
		else:# case where the green line is not detected
			 cX_g = 0
			 cY_g = len(cvImage)//2
			 r = 0
			
		mask_y = cv2.inRange(hsv,(low_y),(up_y))
		M = cv2.moments(mask_y)
		
		
		if M["m00"]>0:
			
			if yellow_flag==0: 
				yellow_flag = 1
				y[0]  = int (M["m10"] / M["m00" ] )
				y[1]  = int (M["m01"] / M["m00" ] )
		else:
			
			if r != l:	
				yellow_flag = 0
				y[0]  = len(cvImage[0])//2
				y[1] = len(cvImage)//2
		cX_y,cY_y = y
		#print(cX_2,cX_1)
		#print(yellow_flag)
		
			
		mask = cv2.bitwise_or(mask_r,mask_g)	#create a mask from the green and red one 
		nex_img = cv2.bitwise_and(cvImage,cvImage,mask=mask)#apply the mask to our image 
		center_img_x = int(len(nex_img[0])/2) # the x center of our image 
		center_img_y = int(len(nex_img))
		
		if (cY_r != None and cY_g!= None  and cX_r != None and cX_g!=None): # if all lines detected 
			lines_mean_y = int((cY_r+cY_g)/2) #compute the mean of our lines position
			lines_mean_x =  int((cX_r +cX_g)/2) +20 # the +20 is an adjustement to make sure that the robot will go right in the roundabout

			nex_img[cY_r-10:cY_r+10,cX_r-10:cX_r+10] =  255 #to put a white square on the red line mean
			nex_img[cY_g-10:cY_g+10,cX_g-10:cX_g+10] =  255 #to put a white square on the green line mean
			#nex_img[cY_y-10:cY_y+10,cX_y-10:cX_y+10] =  255
			#nex_img[lines_mean_y-10:lines_mean_y+10,lines_mean_x-10:lines_mean_x+10] =  255
			#nex_img[center_img_y-100:center_img_y-96,center_img_x-15:center_img_x+19] =  123	
		
		self.arr.data= [lines_mean_y,lines_mean_x,center_img_x,cX_y] #the array that we will pusblish
		self.IMG.display_image(nex_img) #display the image 
		self.pub.publish(self.arr) #publish the data

		
		
		
	def main(self): #where alle the computations are done
		rospy.init_node("cam_get_data", anonymous=True)
		self.rate = rospy.Rate(10) #10 Hz seemed to be a good rate in practice
		self.sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.callback)
		self.pub = rospy.Publisher("/cam_data", UInt16MultiArray, queue_size=1) 
		rospy.spin()




if __name__ == '__main__':
	test = get_data()
	test.main()


