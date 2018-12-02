#!/usr/bin/env python


from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion
from image_geometry import PinholeCameraModel
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
roll = pitch = curr_yaw = axes = 0
first_yaw_radians = "start"
from sensor_msgs.msg import LaserScan
going_forward = True
from sys import exit
can_move = "not initialized"
x_curr = "not initialized"
y_curr = 0
x_first = "not initialized"
y_first = 0
found_obj = "not initialized"
ranges = 0 

	
def check_obstacles(msg): 
	global can_move, ranges	
	ranges = msg.ranges
	if msg.ranges[0] > 0.6: # there's no obstacle in front of us
		can_move = True
	else:
		can_move = False
		
def get_position(msg):
	global x_curr,y_curr, x_first, y_first
	position = msg.pose.pose.position
	if x_first == "not initialized":
		x_first = position.x   
		y_first = position.y 	
	else:
		x_curr = position.x
		y_curr = position.y

def move_forward():
	global x_curr,y_curr, x_first, y_first
	global can_move, ranges
	x_curr = x_first = can_move = "not initialized"
	sub_helper_laser = rospy.Subscriber('/scan', LaserScan, check_obstacles)
	sub_helper_pose = rospy.Subscriber('/odom', Odometry, get_position)

	#rospy.loginfo("Robot starting to move 50cm forward if he can")
	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10)
	move_parametes = Twist()
	

        current_distance = 0
	distance = 0.5
	
	while can_move == "not initialized" or x_first == "not initialized" or x_curr == "not initialized" :
		pass
	
	
	if can_move:
		if ranges[10] < 0.6 : 
			turn_around_helper(15, True)
		if ranges[350] < 0.6 : 
			turn_around_helper(-15, True)
		while (not rospy.is_shutdown()) and current_distance < distance: # we will move till we pass 0.5 m
			
			move_parametes.linear.x = 0.1  
			if (current_distance >0.20):
				move_parametes.linear.x = 0.06
			if (current_distance >0.45):
				move_parametes.linear.x = 0.02
			move_parametes.angular.z = 0	 # no angle
			publisher_handler.publish(move_parametes)
			r.sleep()			
			# distance between 2 points, by fomula			
			current_distance = math.sqrt(math.pow((x_first - x_curr),2) + math.pow( (y_first - y_curr), 2) )
		
			
			
			

		move_parametes.linear.x = 0
		#Force the robot to stop
		publisher_handler.publish(move_parametes)

	else:
		print ("There's an obstacle, robot can't move")

def get_rotation (msg):
	global roll, pitch, curr_yaw, first_yaw_radians 
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	if first_yaw_radians == "start":
		roll, pitch, first_yaw_radians = euler_from_quaternion (orientation_list)		
	else:
		roll, pitch, curr_yaw = euler_from_quaternion (orientation_list)
    	

def turn_around():
    	degrees_to_add = int(input("Insert degrees>>"))
        turn_around_helper(degrees_to_add, False)
	
	
def turn_around_helper(degrees_to_add, do_it_the_short_way):
		
	global roll, pitch, curr_yaw, first_yaw_radians 
	first_yaw_radians = curr_yaw = "start"
	sub_handler = rospy.Subscriber('/odom', Odometry, get_rotation)

	while first_yaw_radians == "start" or curr_yaw == "start" :
		pass
	first_yaw_degrees = 180 * first_yaw_radians / math.pi

	# calculating dest degrees (range is -180 to 180)  
	if (first_yaw_degrees < 0):
		first_yaw_degrees = (first_yaw_degrees+360)

	destination_degrees = first_yaw_degrees - degrees_to_add # we want to move clockwise - opposite to the real degrees 
	while destination_degrees< 0:
		destination_degrees = destination_degrees + 360
	while destination_degrees > 360:
		destination_degrees = destination_degrees - 360
	
	if (destination_degrees>180):
		destination_degrees = destination_degrees - 360
	destination_radians = math.pi * destination_degrees / 180
	

	turn_around_parametes = Twist()  
	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10);
	#rospy.loginfo("Robot starting to turn around")
	
	sign = 1 # define if the robot would turn clock wise
	
	if (do_it_the_short_way):
		if (destination_radians > 0 and curr_yaw > 0 and destination_radians > curr_yaw ):
			sign = -1  # the robot turn clockwise when the value is negative.
		else:
			if (destination_radians > 0 and destination_radians < 1.5  and curr_yaw < 0 ):
				sign = 1
			else:
				if (destination_radians < -1.5  and curr_yaw > 0 and destination_radians > -3 ):
					sign = 1
				else:
					if (destination_radians < 0  and curr_yaw < 0 and destination_radians > curr_yaw ):
						sign = 1

	while not rospy.is_shutdown():			
		turn_around_parametes.angular.z = (0.3 * sign)	
		publisher_handler.publish(turn_around_parametes) 
		r.sleep()
		if abs(destination_radians - curr_yaw) <0.02:
			turn_around_parametes.angular.z = 0 # we stop the robot
			publisher_handler.publish(turn_around_parametes)  	
			break
	

def distance_to_colored_object() :
        color = raw_input("Color is required. Please enter color: ")
        distance_to_colored_object_helper(color)
        

def calc_distance() :
	laserInfo = rospy.wait_for_message('/scan',LaserScan)
	distance = np.inf
	if not laserInfo.ranges[0] == np.inf :
		return laserInfo.ranges[0]
	for x in range(1, 2):	
		if ( laserInfo.ranges[x] < distance ):
			distance = laserInfo.ranges[x]
	for x in range(359, 360):	
		if ( laserInfo.ranges[x] < distance ):
			distance = laserInfo.ranges[x]
        return distance
    
                    
def distance_to_colored_object_helper(color) :
        global found_obj 

        camInfo = rospy.wait_for_message('/camera/camera_info',CameraInfo)
        IMAGE = rospy.wait_for_message('/camera/image_raw',Image)
        camera = PinholeCameraModel()
        camera.fromCameraInfo(camInfo)
	bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(IMAGE,'bgr8')
        gau_blur = cv2.GaussianBlur(cv_image, (3,3), 0)
       
        if (color == 'blue'): 
            lower = np.array([100, 0, 0], dtype="uint8")
            upper = np.array([255, 204, 70], dtype="uint8")
        elif (color == 'red'):
            lower = np.array([0, 0, 100], dtype="uint8")
            upper = np.array([97, 105, 255], dtype="uint8")
        elif (color == 'green'):
            lower = np.array([0, 70, 0], dtype="uint8")
            upper = np.array([90, 236, 170], dtype="uint8")
        else:
            print("invalid color. exit command")
            exit()
        
        mask_image = cv2.inRange(gau_blur, lower, upper)
      
        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))
        maskOpen = cv2.morphologyEx (mask_image ,cv2.MORPH_OPEN,kernelOpen)
        final_mask = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)	        
	
        
        #find the center of object
        img2, contours, h = cv2.findContours(final_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
	if(len(contours)!=0):
                found_obj = True
		cnt = contours[0]
        	M = cv2.moments(cnt)
        	x_center_obj = int(M['m10']/M['m00'])		
        	y_center_obj = 400 
		center_obj = (x_center_obj,y_center_obj)
		image_center = (400, 400) 
		ray_obj = camera.projectPixelTo3dRay(center_obj)
		ray_img = camera.projectPixelTo3dRay(image_center)				
		# angle between two vectors - by formula. We'll get the cos of the angle so we'll do arccos
		radians = math.acos ( np.dot (ray_obj, ray_img) / ( np.linalg.norm(ray_obj) * np.linalg.norm(ray_img) ) ) 
		degrees = math.degrees(radians)

		if ( x_center_obj < image_center[0] ):		
			degrees = 360 - degrees
		

        	turn_around_helper(degrees, True) #turn the robot
		rospy.sleep(1)
		distance = calc_distance()
		print " Distance to object is %s" % distance
		return distance
	else:
                found_obj = False
		print(color + " object doesn't exist. exit command.")
          

  	
	
def find_colored_object():
        global can_move, found_obj
        color = raw_input("Color is required. Please enter color: ")
        if(not(color == 'blue' or color == 'red' or color == 'green')):
            print("invalid color. exit command")
            exit()
                
        while True :
            distance = distance_to_colored_object_helper(color)
	  
            while found_obj == "not initialized":
                pass
	    
            if(found_obj):
		if(distance < 1):
			print ("distance is : %s" % distance)
			exit()
        	move_forward() 
		rospy.sleep(1)      
            else:
		for x in range(0, 4):
			turn_around_helper(90, True) # check all field around the robot
			distance = distance_to_colored_object_helper(color)
			while found_obj == "not initialized":
              			pass
           		if(found_obj):
				if(distance < 1):
					print ("distance is : %s" % distance)
					exit()
				break
		move_forward()
                if not can_move :
			turn_around_helper(100, True)
			rospy.sleep(1)
                    
            

if __name__ == '__main__':
	rospy.init_node('main_node', anonymous=True)
	print ("type the function number you want")
	print (''' 
		1. Move forward (if there is no obstacle that is closer than 50 cm)
		2. Turn around 
		3. Distance to object with color X 	
		4. Find object with color X ''' )
	command_number = int(input(">>"))

	functions = [move_forward, turn_around, distance_to_colored_object, find_colored_object]
	functions[command_number-1]() # executing the relevant command
	
	










