#!/usr/bin/python

#Credits: https://www.safaribooksonline.com/library/view/python-cookbook-3rd/9781449357337/ch01s05.html
#		  https://en.wikipedia.org/wiki/A*_search_algorithm
#		  

import numpy as np
import rospy
import heapq
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import sys

class PQ:
	def __init__(self):
		self.q = []
	
	def isempty(self):
		val = 0
		val = len(self.q)
		return val
	
	def enqueue(self, x, priority):
		#print "andi"
		heapq.heappush(self.q, (priority, x))
	
	def dequeue(self):
		#print "check"
		val = heapq.heappop(self.q)[1]
		return val

	
class Astar:

	def __init__(self,grid, start, goal, xdim, ydim):
		
		self.start = start
		self.goal = goal
		self.grid = grid
		self.xd = xdim
		self.yd = ydim
		self.idx = 0
		
	def get_hscore(self,a,b):
		
		return abs(a[0]-b[0]) + abs(a[1]-b[1])
	
	
	
	def get_cost(self,a,b):
		
		val = self.get_hscore(a,b)
		if(val == 2):
			return 14
		else:
			return 10
	
	def get_ngbr(self, pos):
		(x, y) = pos
		ret_val = {}
		val = []
		for i in range(-1+x, 2+x):
			for j in range(-1+y, 2+y):
				if(i<0 or i>=self.xd or j<0 or j>=self.yd):
					continue
				
				
				if(self.grid[i][j]!=1):
					p = (i,j)
					
					if(self.get_hscore(pos,p) == 2):
						continue
					else:
						val.append((i,j))
					
					#val.append((i,j))
		ret_val=val
		
		return ret_val
	
	def lookup_in(self, cost, pos):
		val = False
		if pos in cost.keys():
			val = True
		return val
	
	def put_mark(self, pos):
		self.grid[pos[0]][pos[1]] = 9
		self.disp()
		
	def search_node(self):
		q = PQ()
		q.enqueue(self.start,0)
		
		source = {}
		cost = {}
		source[self.start] = 0
		cost[self.start] = 0
		
		while(0 != q.isempty()):
			
			curr_node = q.dequeue()
			#self.put_mark(curr_node)
			if(curr_node == self.goal):
				break
			
			ngbr = self.get_ngbr(curr_node)
			#print "ngbr",ngbr
			
			for i in ngbr:
				#print i,"i",curr_node,"curr",cost,"cost"
				cst = cost[curr_node] + self.get_cost(curr_node,i)
				
				if(not self.lookup_in(cost,i) or cst < cost[i]):
					cost[i] = cst
					pr = cst + self.get_hscore(i,self.goal)
					q.enqueue(i, pr)
					source[i] = curr_node
					#print "updating source",source
					
					#self.grid[curr_node[0]][curr_node[1]]=9
					#self.disp()
		return source, cost
		
	def disp(self):
		a=[]
		for i in range(0,self.xd):
			for j in range(0,self.yd):
				a.append(self.grid[i][j])
				
			print a
			a= [] 
	
	def disp_x(self, grid):
		
		a=[]
		for i in range(0,self.xd):
			for j in range(0,self.yd):
				a.append(grid[i][j])
			rospy.loginfo(a)	
			#print a
			a= [] 


class Motion:
	
	def __init__(self, path):
		
		self.path = path
		self.sub = None
		self.pub = None
		self.idx = 0
		self.bot_orien = 0
		self.bot_pos = None
	
	def get_angle_from_orientation(self,rotation):
		quaternion = (rotation.x,rotation.y,rotation.z,rotation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw=euler[2]
		return yaw
		
	def req_orien(self, curr_pos, int_pos):
		dx = round(int_pos[0])-round(curr_pos[0])
		dy = round(int_pos[1])-round(curr_pos[1])
		val = math.atan2(dy,dx)
		
		tmp = val
		f = np.pi/2
		if(dx == 0):
			if(dy < 0):
				f = -f
			elif(dy > 0):
				f = -f
			
		elif(dy == 0):
			if(dx > 0):
				f = -f
			elif(dx < 0):
				f = -f
		elif(dx < 0):
			#Going in negative direction
			if(dy < 0):
				f = -f
			elif(dy > 0):
				f = -f
		elif(dx > 0):
			if(dy < 0):
				f = f
			elif(dy > 0):
				f = f
		val = f+val
		
		#rospy.loginfo("bef"+str(tmp)+"adt"+str(val))
		
		return val
	
	def transform_point(self,pt):
		
		ax = 9
		by = 10
		
		#print "pt",pt
		ypos = ax + pt[0]
		xpos = by - pt[1]
		
		p = (xpos, ypos)
		return p
		
		
	def pose_callback(self, msg):
		
		'''
		ax = 9
		by = 10
		ypos = by - msg.pose.pose.position.y 
		xpos = ax + msg.pose.pose.position.x 
		curr_pos = (xpos,ypos)
		'''
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		
		pt = (x, y)
		curr_pos = self.transform_point(pt)
		self.bot_pos = curr_pos
		self.bot_orien = self.get_angle_from_orientation(msg.pose.pose.orientation)
	
	def move(self, rot, trans, itera, fr,ft, mesg, r):
		
		for k in range(0,itera):
				mesg.linear.x = trans*ft
				mesg.angular.z = rot*fr
				
				self.pub.publish(mesg)
				r.sleep()
			
	def get_dist(self,a,b):
		
		return np.sqrt(pow((a[0]-b[0]),2) + pow((a[1]-b[1]),2))
			
	
	def start_moving(self):
		
		#rospy.loginfo("start_moving")
		mesg = Twist()
		r = rospy.Rate(5)
		#Intended position
		goal_pos = self.path[0]
		final_goal = self.path[-1]
		#for i in range(1,len(self.path)):
		my_pos = self.bot_pos
		i = 0
		thresh_d = 1.5
		while(True):
			
			if(abs(self.get_dist(my_pos,final_goal))<=0.5):
				break
			
			if(i == len(self.path)-1):
				thresh_d = 0.05
				
			if(abs(self.get_dist(my_pos,goal_pos))<=thresh_d):
				i += 1
				if(i == len(self.path)):
					break
				else:
					goal_pos = self.path[i]
			
					
			#my_pos = self.transform_point(self.bot_pos)
			my_pos = self.bot_pos
			calc_orien = self.req_orien(my_pos, goal_pos)
			req_rot = calc_orien - self.bot_orien

			if(req_rot < -np.pi or req_rot > np.pi):
				req_rot = 2*np.pi-abs(req_rot)
		

			
			dx = goal_pos[0]-my_pos[0]
			dy = goal_pos[1]-my_pos[1]
			dist = math.sqrt(pow(dx,2) + pow(dy,2))
			rospy.loginfo("i "+str(i)+" my_pos "+str(my_pos)+" goal_pos "+str(goal_pos)+" req_rot "+str(req_rot)+" my_orien "+str(self.bot_orien)+" dist "+str(dist))
			
			flag = "ROTATE"
			if(abs(dist) > 0.5):
				if(abs(req_rot) > 0.5):
					flag = "TRANSROTATE"
				else:
					flag = "TRANSLATE"
			
			rospy.loginfo(flag)
			
			if(flag == "ROTATE"):			#fr,ft
				self.move(req_rot, 0, 10, 0.5, 0.5, mesg, r)
			elif(flag == "TRANSLATE"):
				self.move(0, dist, 10, 0.5, 0.5, mesg, r)
			else:
				self.move(req_rot, 0, 10, 0.5,0.5, mesg, r)
				self.move(0, dist, 12, 0.5,0.55, mesg, r)
				#self.move(req_rot, dist, 10, 0.42, 0.41, mesg, r)
				'''
				if(i > len(self.path)/2):
					self.move(req_rot, 0, 10, 0.5,0.5, mesg, r)
					self.move(0, dist, 12, 0.65,0.57, mesg, r)
				else:
					self.move(req_rot, dist, 10, 0.42, 0.41, mesg, r)
				'''
			#goal_pos = self.path[i]
			
		
	
	def start_seq(self):
		
		self.sub = rospy.Subscriber("base_pose_ground_truth", Odometry, self.pose_callback)
		self.pub = rospy.Publisher("cmd_vel", Twist,queue_size=10)
		msg = Twist()
		r = rospy.Rate(5)
		for i in range(0,12):
			msg.linear.x = 0
			msg.angular.z = -np.pi/4
			self.pub.publish(msg)
			r.sleep()
		rospy.loginfo("moved")	
		self.start_moving()
		
		
		
		
		

if __name__ == "__main__":
	
	grid=[[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
				[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
				[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
				[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
				[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
				[0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
				[0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
				[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
				[0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
				[0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
				[0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
				[0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
				[0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
				[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
				[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
				[0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
				[0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
				[0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
				[0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
				[0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]
	
	rospy.init_node('astar')
	try:
		
		ar = sys.argv[:]
		#start_point = 
		rospy.loginfo("points")
		
		sx = int(math.floor(float(ar[1])))
		sy = int(math.floor(float(ar[2])))
		gx = int(math.floor(float(ar[3])))
		gy = int(math.floor(float(ar[4])))
	
		rospy.loginfo(str(sx)+str(sy)+str(gx)+str(gy))
		
		ax = 9
		by = 10
		#start = (-8 + x_off, 2 + y_off)
		#dest = (4 + x_off, 9 + y_off)
		
		#start = (by -(-2) , -8 + ax)	#12,1
		#dest = (by - 9, 4 + ax)     #1,13
		
		start = (by - sy , sx + ax)	#12,1
		dest = (by - gy, gx + ax)     #1,13
		
		
		grid[start[0]][start[1]] = 9
		#grid[dest[0]][dest[1]] = 9
		tmp = grid
		
		ob = Astar(grid, start, dest, 20, 18)
		#ob.disp()
		src, cst = ob.search_node()
		#print src,"src"
		#rospy.loginfo(str(src)+"src")
		rospy.loginfo("start"+str(start)+"dest"+str(dest))
		val = src[dest]
		path = []
		while(val != start):
			path.append(val)
			val = src[val]
			
		#print "path ####", path,"len", len(path)
		for p in range(0,len(path)):
			#print p
			pt = path[p]
			tmp[pt[0]][pt[1]] = 9 
		
		ob.disp_x(tmp)
		path.reverse()
		rospy.loginfo(str(path)+"path")
		m = Motion(path)
		m.start_seq()
		
		
	except rospy.ROSInterruptException:
		pass

	
