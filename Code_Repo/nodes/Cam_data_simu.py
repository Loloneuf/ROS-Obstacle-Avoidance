#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16MultiArray
from image_tools import ImageTools
import cv_bridge
import cv2

class get_data:  # the class format allow us to have a cleaner code structure as well as a an efficient way to save our variables
	def __init__(self):
		print("initalizing publisher and subscriber")
		self.arr=UInt16MultiArray() #the object that wil store what we will publish
		self.cvBridge = cv_bridge.CvBridge() #the object that will store our image
		self.IMG = ImageTools() # an object from a usefull library that makes easier the conversion between types
		self.image = []


	def callback(self,msg):
		self.image = self.IMG.convert_ros_compressed_to_cv2(msg) 


	def main(self):
		rospy.init_node("cam_test", anonymous=True)
		self.rate = rospy.Rate(10)#10 Hz seemed to be a good rate in practice
		self.sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.callback)
		self.pub = rospy.Publisher("/cam_data", UInt16MultiArray, queue_size=1) 
		
		while not rospy.is_shutdown():
			if self.image != []:
				cvImage1 = self.image
				cvImage = cvImage1[len(cvImage1)//2::,5:len(cvImage1[0])-5] # crop to only get the low part of the camera
				hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV) #convert our image in a HSV format

					
					
				#white filter	
				low_w =np.array([0,0,220])
				up_w =np.array([180,40,255])

				#yellow filter
				low_y =np.array([15,200,200])
				up_y =np.array([40,255,255])
				
				
				'''
				#blue
				low_b =np.array([95,40,100])
				up_b =np.array([140,250,255])    '''
						
				mask_w= cv2.inRange(hsv,(low_w),(up_w))# create a mask to use several operator on it 		
				M = cv2.moments(mask_w) #get the moment to compute the mean of our lines
				lines_mean_x = 0 #variable that will be use to store the x mean between our colored lines
				lines_mean_y = 0

				if M["m00"]>0:
					cX_w = int (M["m10"] / M["m00" ] )
					cY_w = int (M["m01"] / M["m00" ] )

				else:# case where the white line is not detected

					cX_w = len(cvImage[0])
					cY_w = len(cvImage)//2

				mask_y = cv2.inRange(hsv,(low_y),(up_y))
				M = cv2.moments(mask_y)

				if M["m00"]>0:
					 cX_y = int (M["m10"] / M["m00" ] )
					 cY_y = int (M["m01"] / M["m00" ] )

				else:# case where the yellow line is not detected
					 cX_y= 0
					 cY_y = len(cvImage)//2
					
				mask = cv2.bitwise_or(mask_w,mask_y)	#create a mask from the green and red one 
				nex_img = cv2.bitwise_and(cvImage,cvImage,mask=mask)#apply the mask to our image 
				center_img_x = int(len(nex_img[0])/2) # the x center of our image 
				center_img_y = int(len(nex_img))
				
				
				if (cY_w != None and cY_y!= None  and cX_w != None and cX_y!=None): # if all lines detected 
					lines_mean_y = int((cY_w+cY_y)/2) #compute the mean of our lines position
					lines_mean_x =  int((cX_w +cX_y)/2) +15 # the +10 is an adjustement to make sure that the robot will go right in the roundabout

					nex_img[cY_w-10:cY_w+10,cX_w-10:cX_w+10] =  255 #to put a white square on the white line mean
					nex_img[cY_y-10:cY_y+10,cX_y-10:cX_y+10] =  255 #to put a white square on the yellow line mean
					nex_img[lines_mean_y-10:lines_mean_y+10,lines_mean_x-10:lines_mean_x+10] =  255 #same for the lines mean

					
				self.arr.data= [lines_mean_y,lines_mean_x,center_img_x] #the array that we will pusblish
				self.IMG.display_image(nex_img) #display the image 
				self.pub.publish(self.arr) #publish the data
				self.rate.sleep() #to set the publishing rate to our rate that we chose earlier
			
		rospy.spin()


if __name__ == '__main__':
	test = get_data()
	test.main()


