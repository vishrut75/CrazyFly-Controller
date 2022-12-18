#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin, acos
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
class Quadrotor():
	def __init__(self):
	    	# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=1)
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback, queue_size=1)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)
		# TODO: include initialization codes if needed
		self.x_req = [0,0,0,0]
		self.y_req = [0,0,0,0]
		self.z_req = [0,0,0]
		global omg_cap 
		omg_cap = 0
	def traj_evaluate(self):
		# TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations
		t = self.t
		t2 = t*t
		t3 = t2*t
		t4 = t3*t
		t5 = t4*t
		if t<5:
			x1=0
			x2=0
			x3=0
			x4=0
		elif t<20:
			x1 = (4664065662093477*t5)/590295810358705651712 - t4/2025 + (22*t3)/2025 - (8*t2)/81 + (32*t)/81 - 47/81
			x2 = (23320328310467385*t4)/590295810358705651712 - (4*t3)/2025 + (22*t2)/675 - (16*t)/81 + 32/81
			x3 = (23320328310467385*t3)/147573952589676412928 - (4*t2)/675 + (44*t)/675 - 16/81
			x4 = (23320328310467385*3*t2)/147573952589676412928 - 8*t/675 + 44/675
		elif t<35:
			x1 = 1
			x2 = 0
			x3 = 0
			x4 = 0
		elif t<50:
			x1 = - (4664065662093477*t5)/590295810358705651712 + (17*t4)/10125 - (286*t3)/2025 + (476*t2)/81 - (9800*t)/81 + 80000/81
			x2 = - (23320328310467385*t4)/590295810358705651712 + (68*t3)/10125 - (286*t2)/675 + (952*t)/81 - 9800/81
			x3 = - (23320328310467385*t3)/147573952589676412928 + (68*t2)/3375 - (572*t)/675 + 952/81
			x4 = - (23320328310467385*3*t2)/147573952589676412928 + (136*t)/3375 - (572)/675
		else:
			x1 = 0
			x2 = 0
			x3 = 0
			x4 = 0

		if t<20:
			y1 = 0
			y2 = 0
			y3 = 0
			y4 = 0
		elif t<35:
			y1 = (4664065662093477*t5)/590295810358705651712 - (11*t4)/10125 + (118*t3)/2025 - (616*t2)/405 + (1568*t)/81 - 7808/81
			y2 = (23320328310467385*t4)/590295810358705651712 - (44*t3)/10125 + (118*t2)/675 - (1232*t)/405 + 1568/81
			y3 = (23320328310467385*t3)/147573952589676412928 - (44*t2)/3375 + (236*t)/675 - 1232/405
			y4 = (23320328310467385*3*t2)/147573952589676412928 - (88*t)/3375 + (236)/675
		elif t<50:
			y1 = 1
			y2 = 0
			y3 = 0
			y4 = 0
		elif t<65:
			y1 = - (4664065662093477*t5)/590295810358705651712 + (23*t4)/10125 - (526*t3)/2025 + (1196*t2)/81 - (33800*t)/81 + 5159302209836171/1099511627776
			y2 = - (23320328310467385*t4)/590295810358705651712 + (92*t3)/10125 - (526*t2)/675 + (2392*t)/81 - 33800/81
			y3 = - (23320328310467385*t3)/147573952589676412928 + (92*t2)/3375 - (1052*t)/675 + 2392/81
			y4 = - (23320328310467385*3*t2)/147573952589676412928 + (92*2*t)/3375 - (1052)/675
		else :
			y1 = 0
			y2 = 0
			y3 = 0
			y4 = 0

		

		if t<5:
			z1 = (6*t5)/3125 - (3*t4)/125 + (2*t3)/25
			z2 = (6*t4)/625 - (12*t3)/125 + (6*t2)/25
			z3 = (24*t3)/625 - (36*t2)/125 + (12*t)/25
		else:
			z1 = 1
			z2 = 0
			z3 = 0
		self.x_req = [x1,x2,x3,x4]
		self.y_req = [y1,y2,y3,y4]
		self.z_req = [z1,z2,z3]


	def traj_eval(self):

		t = self.t
		t2 = t*t
		t3 = t2*t
		t4 = t3*t
		t5 = t4*t
		if t<5:
			x1=0
			x2=0
			x3=0
			x4=0
		elif t<15:
			x1 = (3*t5)/50000 - (3*t4)/1000 + (11*t3)/200 - (9*t2)/20 + (27*t)/16 - 19/8
			x2 = (3*t4)/10000 - (3*t3)/250 + (33*t2)/200 - (9*t)/10 + 27/16
			x3 = (3*t3)/2500 - (9*t2)/250 + (33*t)/100 - 9/10
			x4 = (9*t2)/2500 - (18*t)/250 + (33)/100
		elif t<25:
			x1 = 1
			x2 = 0
			x3 = 0
			x4 = 0
		elif t<35:
			x1 = - (3*t5)/50000 + (9*t4)/1000 - (107*t3)/200 + (63*t2)/4 - (3675*t)/16 + 10633/8
			x2 = - (3*t4)/10000 + (9*t3)/250 - (321*t2)/200 + (63*t)/2 - 3675/16
			x3 = - (3*t3)/2500 + (27*t2)/250 - (321*t)/100 + 63/2
			x4 = - (9*t2)/2500 + (54*t)/250 - (321)/100
		else:
			x1 = 0
			x2 = 0
			x3 = 0
			x4 = 0
			

		if t<15:
			y1=0
			y2=0
			y3=0
			y4=0
		elif t<25:
			y1 = (3*t5)/50000 - (3*t4)/500 + (47*t3)/200 - (9*t2)/2 + (675*t)/16 - 621/4
			y2 = (3*t4)/10000 - (3*t3)/125 + (141*t2)/200 - 9*t + 675/16
			y3 = (3*t3)/2500 - (9*t2)/125 + (141*t)/100 - 9
			y4 = (9*t2)/2500 - (18*t)/125 + (141)/100
		elif t<35:
			y1 = 1
			y2 = 0
			y3 = 0
			y4 = 0
		elif t<45:
			y1 = - (3*t5)/50000 + (3*t4)/250 - (191*t3)/200 + (189*t2)/5 - (11907*t)/16 + 5832
			y2 = - (3*t4)/10000 + (6*t3)/125 - (573*t2)/200 + (378*t)/5 - 11907/16
			y3 = - (3*t3)/2500 + (18*t2)/125 - (573*t)/100 + 378/5
			y4 = - (9*t2)/2500 + (36*t)/125 - (573)/100
		else:
			y1 = 0
			y2 = 0
			y3 = 0
			y4 = 0
		

		if t<5:
			z1 = (6*t5)/3125 - (3*t4)/125 + (2*t3)/25
			z2 = (6*t4)/625 - (12*t3)/125 + (6*t2)/25
			z3 = (24*t3)/625 - (36*t2)/125 + (12*t)/25
		elif t<45:
			z1 = 1
			z2 = 0
			z3 = 0
		elif t<65:
			z1 = - (8854437155380585*t5)/4722366482869645213696 + (33*t4)/64000 - (359*t3)/6400 + (3861*t2)/1280 - (41067*t)/512 + 432809/512
			z2 = - (44272185776902925*t4)/4722366482869645213696 + (33*t3)/16000 - (1077*t2)/6400 + (3861*t)/640 - 41067/512
			z3 = - (44272185776902925*t3)/1180591620717411303424 + (99*t2)/16000 - (1077*t)/3200 + 3861/640
		else:
			z1 = 0
			z2 = 0
			z3 = 0
		

		self.x_req = [x1,x2,x3,x4]
		self.y_req = [y1,y2,y3,y4]
		self.z_req = [z1,z2,z3]

	def traj_sin(self):

		t = self.t
		t2 = t*t
		t3 = t2*t
		t4 = t3*t
		t5 = t4*t
		if t<5:
			x1=0
			x2=0
			x3=0
			x4=0
		elif t<15:
			d0 = 0
			df = 1
			t0 = 5
			tf = 15
			dt = tf-t0
			b = 2*pi/dt
			A = 9*b*(df-d0)/(8*dt)
			c = b*t0
			x3 = A*sin(b*t-c)
			x2 = -1*(A/b)*cos(b*t-c) + (A/b)
			x1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0
			b = 3*b
			A = A/3
			c = b*t0
			x3 = x3 - A*sin(b*t-c)
			x2 = x2 + (A/b)*cos(b*t-c) - (A/b)
			x1 = x1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0)
		elif t<25:
			x1 = 1
			x2 = 0
			x3 = 0
			x4 = 0
		elif t<35:
			d0 = 1
			df = 0
			t0 = 25
			tf = 35
			dt = tf-t0
			b = 2*pi/dt
			A = 9*b*(df-d0)/(8*dt)
			c = b*t0
			x3 = A*sin(b*t-c)
			x2 = -1*(A/b)*cos(b*t-c) + (A/b)
			x1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0
			b = 3*b
			A = A/3
			c = b*t0
			x3 = x3 - A*sin(b*t-c)
			x2 = x2 + (A/b)*cos(b*t-c) - (A/b)
			x1 = x1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0)
		else:
			x1 = 0
			x2 = 0
			x3 = 0
			x4 = 0
			

		if t<15:
			y1=0
			y2=0
			y3=0
			y4=0
		elif t<25:
			d0 = 0
			df = 1
			t0 = 15
			tf = 25
			dt = tf-t0
			b = 2*pi/dt
			A = 9*b*(df-d0)/(8*dt)
			c = b*t0
			y3 = A*sin(b*t-c)
			y2 = -1*(A/b)*cos(b*t-c) + (A/b)
			y1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0
			b = 3*b
			A = A/3
			c = b*t0
			y3 = y3 - A*sin(b*t-c)
			y2 = y2 + (A/b)*cos(b*t-c) - (A/b)
			y1 = y1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0)
		elif t<35:
			y1 = 1
			y2 = 0
			y3 = 0
			y4 = 0
		elif t<45:
			d0 = 1
			df = 0
			t0 = 35
			tf = 45
			dt = tf-t0
			b = 2*pi/dt
			A = 9*b*(df-d0)/(8*dt)
			c = b*t0
			y3 = A*sin(b*t-c)
			y2 = -1*(A/b)*cos(b*t-c) + (A/b)
			y1 = -1*(A/(b*b))*sin(b*t-c) + (A/b)*(t-t0) + d0
			b = 3*b
			A = A/3
			c = b*t0
			y3 = y3 - A*sin(b*t-c)
			y2 = y2 + (A/b)*cos(b*t-c) - (A/b)
			y1 = y1 + (A/(b*b))*sin(b*t-c) - (A/b)*(t-t0)
		else:
			y1 = 0
			y2 = 0
			y3 = 0
			y4 = 0
		

		if t<5:
			z1 = (6*t5)/3125 - (3*t4)/125 + (2*t3)/25
			z2 = (6*t4)/625 - (12*t3)/125 + (6*t2)/25
			z3 = (24*t3)/625 - (36*t2)/125 + (12*t)/25
		elif t<45:
			z1 = 1
			z2 = 0
			z3 = 0
		elif t<65:
			z1 = - (8854437155380585*t5)/4722366482869645213696 + (33*t4)/64000 - (359*t3)/6400 + (3861*t2)/1280 - (41067*t)/512 + 432809/512
			z2 = - (44272185776902925*t4)/4722366482869645213696 + (33*t3)/16000 - (1077*t2)/6400 + (3861*t)/640 - 41067/512
			z3 = - (44272185776902925*t3)/1180591620717411303424 + (99*t2)/16000 - (1077*t)/3200 + 3861/640
		else:
			z1 = 0
			z2 = 0
			z3 = 0
		

		self.x_req = [x1,x2,x3,0]
		self.y_req = [y1,y2,y3,0]
		self.z_req = [z1,z2,z3]


	def sat(self,val,phi):
		sn = 0
		if val < 0:
			sn = -1
		elif val > 0:
			sn = 1

		if abs(val)>phi:
			return sn
		return val/phi

	def getVel(self, xyz, xyz_dot, rpy, rpy_dot):
		m = 27*1e-3
		l = 46*1e-3
		Ix = 16.571710*1e-6
		Iy = 16.571710*1e-6
		Iz = 29.261652*1e-6
		Ip = 12.65625*1e-8
		KF = 1.28192*1e-8
		KM = 5.964552*1e-3
		g = 9.81
		lmbda = [4,6,6,16]
		K = [10,400,450,8]
		global omg_cap
		drag = 0 # 0.2
		ar = l*l/m
		rho = 0.2*omg_cap
		rhozxy = 0 # 2.5*1e-6
		rhoxyz = 0 # 6.6*1e-6
		rhop = rho*15*1e-8
		rhoi = 0 # 3.3*1e-6

		ak = 1/(4*KF)
		bk = sqrt(2)*ak/l

		phi = rpy[0,0]
		tta = rpy[1,0]
		shi = rpy[2,0]
		cp = cos(phi)
		sp = sin(phi)
		ct = cos(tta)
		st = sin(tta)
		cs = cos(shi)
		ss = cos(shi)

		xd = self.x_req[0]
		xd_dot = self.x_req[1]
		xd_ddot = self.x_req[2] 
		xd_dddot = self.x_req[3] 
		yd = self.y_req[0] 
		yd_dot = self.y_req[1]
		yd_ddot = self.y_req[2] 
		yd_dddot = self.y_req[3]
		zd = self.z_req[0] 
		zd_dot = self.z_req[1] 
		zd_ddot = self.z_req[2]


		phid2 = 0
		phid3 = 0
		ttad2 = 0
		ttad3 = 0

		x1 = xyz[0,0] 
		y1 = xyz[1,0] 
		z1 = xyz[2,0] 
		phi1 = rpy[0,0] 
		tta1 = rpy[1,0] 
		shi1 = rpy[2,0] 
		x2 = xyz_dot[0,0] 
		y2 = xyz_dot[1,0] 
		z2 = xyz_dot[2,0] 
		phi2 = rpy_dot[0,0] 
		tta2 = rpy_dot[1,0] 
		shi2 = rpy_dot[2,0]



		ez1 = zd - z1
		ez2 = zd_dot - z2
		s1 = ez2 + lmbda[0]*ez1
		u1 = m*(lmbda[0]*ez2 + zd_ddot + g + drag*ar*z1*abs(z1) + K[0]*self.sat(s1,0.1))/(cos(phi1)*cos(tta1))

		# print("u1/m", abs(u1)/m)

		Kpx = 30*(1 + 0.01*abs(u1)/m) #30
		Kdx = 11*(1 + 0.01*abs(u1)/m) #11
		Kpy = 56*(1 + 0.01*abs(u1)/m) #42
		Kdy = 15*(1 + 0.01*abs(u1)/m) #13
		arx = ar*st
		ary = ar*sp
		ex1 = xd - x1
		ex2 = xd_dot - x2
		ey1 = yd - y1
		ey2 = yd_dot - y2
		uxd = xd_ddot + Kpx*ex1 + Kdx*ex2 + drag*arx*x1*abs(x1)
		uyd = yd_ddot + Kpy*ey1 + Kdy*ey2 - drag*ary*y1*abs(y1)


		x3 = 0 # 0.5*(1/m)*(cos(phi1)*sin(tta1)*cos(shi1) + sin(phi1)*sin(shi1))*u1
		y3 = 0 # 0.5*(1/m)*(cos(phi1)*sin(tta1)*sin(shi1) - sin(phi1)*cos(shi1))*u1
		ex3 = xd_ddot - x3
		ey3 = xd_ddot - y3
		duxd = xd_dddot + Kpx*ex2 + Kdx*ex3
		duyd = yd_dddot + Kpy*ey2 + Kdy*ey3

		if u1!=0 :
			Fxd = m*uxd/u1
			Fyd = m*uyd/u1

			Fpd = Fxd*sin(shi1) - Fyd*cos(shi1)
			Ftd = (Fyd*sin(shi1) + Fxd*cos(shi1))/cp

			dFxd = m*duxd/u1
			dFyd = m*duyd/u1

			dFpd = dFxd*sin(shi1) - dFyd*cos(shi1) + shi2*(Ftd*cp)
			dFtd = ((dFyd*sin(shi1) + dFxd*cos(shi1) - shi2*Fpd)*cp + Ftd*sp)/(cp*cp)

			if(abs(Fpd)>0.95):
				Fpd = 0.95*Fpd/abs(Fpd)
				print("Fpd",Fpd)
			if(abs(Ftd)>0.95):
				Ftd = 0.95*Ftd/abs(Ftd)
				print("Ftd",Ftd)
			# print(Fxd,Fyd)
		else :
			Fpd = 0
			Ftd = 0

		phid1 = asin(Fpd)
		phid2 = dFpd/cos(phi1)
		ephi1 = phid1 - phi1
		ephi2 = phid2 - phi2
		s2 = ephi2 + lmbda[1]*sin(ephi1)
		vre2 = abs(tta2*shi2)*rhozxy + abs(tta2)*rhop + lmbda[1]*rhoi*abs(ephi2) + K[1]*Ix
		vr2 = vre2*self.sat(s2,1)
		u2 = vr2 + Ix*lmbda[1]*cos(ephi1)*ephi2 - tta2*shi2*(Iy-Iz) + Ip*omg_cap*tta2

		ttad1 = asin(Ftd)
		ttad2 = dFtd/cos(tta1)
		etta1 = ttad1 - tta1
		etta2 = ttad2 - tta2
		s3 = etta2 + lmbda[2]*sin(etta1)
		vre3 = abs(phi2*shi2)*rhozxy + abs(phi2)*rhop + lmbda[2]*rhoi*abs(etta2) + K[2]*Iy
		vr3 = vre3*self.sat(s3,1)
		u3 =  vr3 + Iy*lmbda[2]*cos(etta1)*etta2 - phi2*shi2*(Iz-Ix) - Ip*omg_cap*phi2

		shid1 = 0
		eshi1 = shid1 - shi1
		eshi2 = 0 - shi2
		s4 = eshi2 + lmbda[3]*eshi1
		vre4 = K[3]*Iz + abs(phi2*tta2)*rhoxyz
		vr4 = vre4*self.sat(s4,0.01)
		u4 = Iz*(lmbda[3]*eshi2) + vr4
		# print("S:")
		# print(s1,s2,s3,s4)

		# print(ephi1*100//1,etta1*100//1,eshi1*100//1)

		u = np.array([[u1],[u2],[u3],[u4]])
		Alloc = np.array([[ak,    -bk,    -bk,    -ak/KM],[ak,     -bk,    bk,     ak/KM],[ak,     bk,     bk,     -ak/KM],[ak,     bk,     -bk,    ak/KM]])


		w = np.matmul(Alloc,u)
		# print(w)
		vel_sq = np.zeros([4,1])
		vel_sq[0,0] = max(ak*u1 - bk*(u2+u3) - (ak*u4/KM),0)
		vel_sq[1,0] = max(ak*u1 - bk*(u2-u3) + (ak*u4/KM),0)
		vel_sq[2,0] = max(ak*u1 + bk*(u2+u3) - (ak*u4/KM),0)
		vel_sq[3,0] = max(ak*u1 + bk*(u2-u3) + (ak*u4/KM),0)
		motor_vel = np.zeros([4,1])
		motor_vel[0,0] = sqrt(vel_sq[0,0])
		motor_vel[1,0] = sqrt(vel_sq[1,0])
		motor_vel[2,0] = sqrt(vel_sq[2,0])
		motor_vel[3,0] = sqrt(vel_sq[3,0])

		omg_cap = motor_vel[0,0] - motor_vel[1,0] + motor_vel[2,0] - motor_vel[3,0]

		return motor_vel


	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		# obtain the desired values by evaluating the corresponding trajectories
		self.traj_evaluate()
		while(rpy[0,0]>pi):
			rpy[0,0] = rpy[0,0] - 2*pi
		while(rpy[0,0]<-pi):
			rpy[0,0] = rpy[0,0] + 2*pi
		while(rpy[1,0]>pi):
			rpy[1,0] = rpy[1,0] - 2*pi
		while(rpy[1,0]<-pi):
			rpy[1,0] = rpy[1,0] + 2*pi
		while(rpy[2,0]>pi):
			rpy[2,0] = rpy[2,0] - 2*pi
		while(rpy[2,0]<-pi):
			rpy[2,0] = rpy[2,0] + 2*pi
		# publish the motor velocities to the associated ROS topic
		motor_vel = self.getVel(xyz, xyz_dot, rpy, rpy_dot)
		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
		self.motor_speed_pub.publish(motor_speed)


	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0
		
		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
		q = msg.pose.pose.orientation
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		T[0:3, 3] = xyz[0:3, 0]
		R = T[0:3, 0:3]
		xyz_dot = np.dot(R, v_b)
		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
		rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
		rpy = np.expand_dims(rpy, axis=1)
		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])
		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path
		with open("/home/loser/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
			self.mutex_lock_on = True
			pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)
if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
