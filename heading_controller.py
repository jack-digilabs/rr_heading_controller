#!/usr/bin/env python

# Description: This script converts Joystick commands into Joint Velocity commands
# Four buttons: emergency stop button (button B/red), prevents further joystick 
# commands and cancels any existing move_base command
# sends zeros to cmd_vel while in ESTOP state, 
# can be toggled by pressing A button (green) after debounce time
# Monitors X and Y buttons and toggles their state (False on startup) publishes 
# a latched Bool() for X, Y and B buttons as /joystick/<x_, y_ and e_stop>_button

# Xbox controller mapping:
#   axes: [l-stick horz,l-stick vert, l-trigger, r-stick horz, r-stick vert, r-trigger]
#   buttons: [a,b,x,y,lb,rb,back,start,xbox,l-stick,r-stick,l-pad,r-pad,u-pad,d-pad]  

import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalID
import os
import time
import math
from std_msgs.msg import Bool

pid_controller = None

# cmd_vel publisher  
pub_cmd_vel = rospy.Publisher('/cmd_vel/managed', Twist, queue_size=1)
# Constants
PID_RATE = 20.0
P_TERM = 10.0
I_TERM = 10.0
D_TERM = 1/PID_RATE
HEADING_LIST = [0, 180] #, 180, 270, 0, 180]
heading_index = 0
heading_count = 0   
def scaleEulerError(r, sensor):
	e = r - sensor
	if e > 180:
		return (360-e)/180.0
	if e < -180:
		return (360+e)/180.0
	return e/180.0	 

def checkRange(setpoint, heading):
	if (0.95*setpoint <heading <= 1.05*setpoint):
		return True
	return False
			
def startControllerCB(y_button):
	global pid_controller
	pid_controller.running = y_button.data

def imuDataCB(point):
	global pid_controller
	pid_controller.data = point.x

def cmdVelCB(event):
	global pid_controller
	global heading_index
	global heading_count
	setpoint = HEADING_LIST[heading_index]
	heading = pid_controller.data
	cmd = Twist()		
	dt = 1.0/ PID_RATE
	if pid_controller.running:
		e = (setpoint- heading) #scale e to be between 0-1
		s_e = scaleEulerError(setpoint, heading)
		if not (abs(s_e)<0.005):
			#rospy.loginfo("e (deg): %f", s_e)
			u = pid_controller.calcPfb(s_e, dt)
			u_i = pid_controller.calcIfb(s_e, dt)
			u_d = pid_controller.calcDfb(s_e, dt)
			s_u = pid_controller.outputScaleShift(u+u_i + u_d)
			#print(pid_controller.i_last)
			rospy.loginfo("e: %3.2f, u_p: %3.2f, u_i: %3.2f, u_d: %3.2f, s_u: %3.2f", s_e, u, u_i, u_d, s_u)
			cmd.angular.z = s_u #2
			pub_cmd_vel.publish(cmd)
		else:
			cmd.angular.z = 0
			pub_cmd_vel.publish(cmd)
						
			with open('/home/ubuntu/Desktop/data.txt', 'a') as f:
				data_str = '%f,%3.2f,%3.2f,%3.2f\n' %(rospy.get_time(), (heading_count+1), setpoint, heading)
				f.write(data_str)
				
			rospy.loginfo("Completed setpoint %3.2f.", HEADING_LIST[heading_index])
			heading_count = heading_count + 1
			heading_index = (heading_index+1)%len(HEADING_LIST)
			rospy.loginfo("Completed %i setpoints. Next setpoint is %3.2f.", heading_count, HEADING_LIST[heading_index])				
			rospy.sleep(5)

class PIDcontroller:
	e_last = 0.0
	i_last = 0.0
	t_last = 0.0
	u_max = 0.10
	u_min = -0.10
	u_deadzone_high = 0.001
	u_deadzone_low = -0.001
	u_stop_band = 0.1
	data = 0.0
	running = False
	
	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.run = False
		
	#in low speed mode, need at least 1 or -1 for movement
	def outputScaleShift(self, u):
		if u>=self.u_max:
			u = self.u_max
			self.i_last = 0.0
			return -u
		if u<=(self.u_min):
			u = (self.u_min)
			self.i_last = 0.0
			return -u
		if (u>=0.0) and (u<self.u_deadzone_high):
			u = (u+self.u_deadzone_high)
			return -u
		if (u<=0.0) and (u>self.u_deadzone_low):
			u = (u+self.u_deadzone_low)
			return -u
		return -u
			
	def calcDt(self, time):
		dt = time-self.t_last
		print(dt)
		self.t_last = time
		return dt
	
	def calcPfb(self, e, dt):
		return (self.kp * e)
		
	def calcIfb(self, e, dt):
		self.i_last = self.i_last + self.ki * e * dt
		return self.i_last
		
	def calcDfb(self, e, dt):
		return self.kd * (e-self.e_last)/dt
		
	def calcFeedback(self, e, dt):
		P = self.calcPfb(e)
		#I = self.calcIfb(e, dt)
		D = self.calcDfb(e, dt)
		u = P + D
		e_last = e
		return u
		

# Main Function
def heading_controller_main():
	global pid_controller
	pid_controller = PIDcontroller(P_TERM, I_TERM, D_TERM)
	# Initialize driver node
	rospy.init_node('heading_controller_node', anonymous=True)
	rospy.loginfo("Starting heading controller")
	#r = rospy.Rate(20) # 10hz
	# Initialize PID controller timer
	rospy.Timer(rospy.Duration(1.0/PID_RATE), cmdVelCB)
	
	#Write header to file
	with open('/home/ubuntu/Desktop/data.txt', 'w') as f:
		f.write("time,setpoint#,setpoint,actual\n")
	
	while not rospy.is_shutdown():
	# Subscribe to the joystick topic
		sub_cmds = rospy.Subscriber("joystick/y_button", Bool, startControllerCB)
		sub_imu = rospy.Subscriber("imu/euler", Point, imuDataCB)
		pid_controller.running = True
        # spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
		#r.sleep()
    
if __name__ == '__main__':
    try:
        heading_controller_main()
    except rospy.ROSInterruptException:
        pass

