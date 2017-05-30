#!/usr/bin/python
from __future__ import division
import rosbag
import rospy
import tf
from lab4.msg import Motion
import scipy.ndimage.filters as fi
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class BayesianFil():
	def __init__(self,fname,px,py,pz,bel):
		bag=rosbag.Bag(fname)
		self.motion_sig=2
		self.orien_sig=1
		self.gauss1d=self.gkern1d(3,nsig=self.motion_sig)
		self.gauss2d=self.gkern2d(3,nsig=self.orien_sig)
		self.bag=bag
		self.px=px
		self.py=py
		self.pz=pz
		self.bel=bel
		self.tdim=35
		self.dim=3
		self.odim=36
		self.discretization=np.pi/2
		self.pub=rospy.Publisher('visualization_marker',Marker, queue_size=100)
		self.tag=self.assign_tags()
		self.count=0
		
	
	def assign_tags(self):
		
		tags={0:[int(125/20),int(525/20)], 1:[int(125/20),int(325/20)], 2:[int(125/20),int(125/20)], 3:[int(425/20),int(125/20)], 4:[int(425/20),int(325/20)], 5:[int(425/20),int(525/20)]}
		return tags
		
	def receive_msg(self,msg):
		pass
		
	def get_prob_given(self,z,x):
		pass
	
	def get_prob_action(self,x,u):
		pass
		
	def get_angle_from_rotation(self,rotation):
		quaternion = (rotation.x,rotation.y,rotation.z,rotation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw=euler[2]
		#print 'yaw'+str(yaw)
		
		return yaw
	
	def get_estimated_pos(self,mat):
		max_val=-999
		idx=jdx=kdx=0
		
		for i in range(0,self.dim):
			for j in range(0,self.dim):
				for k in range(0,self.odim):
					if(mat[i][j][k]>max_val):
						max_val=mat[i][j][k]
						idx=i
						jdx=j
						kdx=k
		return idx,jdx,kdx
			
	
	def get_curr_ang(self,i,j):
		
		max_val=-999
		
		pos=0
		for k in range(0,self.odim):
			if(self.bel[i][j][k]>max_val):
				max_val=self.bel[i][j][k]
				pos=k
		
		#print "pos: "+str(pos)+" value: "+str(max_val)
		return max_val,pos
				
	def gkern2d(self,kernlen, nsig):
		"""Returns a 2D Gaussian kernel array."""
		
		inp = np.zeros((kernlen, kernlen))
		# set element at the middle to one, a dirac delta
		inp[kernlen//2, kernlen//2] = 1
		# gaussian-smooth the dirac, resulting in a gaussian filter mask
		return fi.gaussian_filter(inp, nsig)
	
	def gkern3d(self,kernlen, nsig):
		"""Returns a 2D Gaussian kernel array."""
		
		inp = np.zeros((kernlen, kernlen,kernlen))
		# set element at the middle to one, a dirac delta
		inp[kernlen//2, kernlen//2,kernlen//2] = 1
		# gaussian-smooth the dirac, resulting in a gaussian filter mask
		return fi.gaussian_filter(inp, nsig)
	
	def gkern1d(self,kernlen, nsig):
		
		
		inp = np.zeros((kernlen))
		# set element at the middle to one, a dirac delta
		inp[kernlen//2] = 1
		# gaussian-smooth the dirac, resulting in a gaussian filter mask
		return fi.gaussian_filter(inp, nsig)
	
	def update_pos(self):
		
		idx=kdx=jdx=0;
		max_val=-99999
		for i in range(0,self.dim):
			for j in range(0,self.dim):
				for k in range(0,self.odim):
					if(self.bel[i][j][k]>max_val):
						max_val=self.bel[i][j][k]
						print "max_val: "+str(max_val)+" at <"+str(i)+","+str(j)+","+str(k)+">"
						idx=i
						jdx=j
						kdx=k
		self.px=idx
		self.py=jdx
		self.pz=kdx			
		


	def distribute_new_pos(self,new_pos_x,new_pos_y,new_pos_z,val):
		
		new_grid=np.zeros((self.tdim,self.tdim,self.odim),dtype='float')
		#sd_pos=3;
		#sd_orien=3;
		mean_pos_x=new_pos_x
		mean_pos_y=new_pos_y
		mean_pos_z=new_pos_z
		
		#small_grid=new_grid[]
		sub_grid=val*self.gkern2(3,2)
		#print sub_grid
		#print new_pos_x,new_pos_y,new_pos_z
		for i in range(new_pos_x-1,new_pos_x+1):
			for j in range(new_pos_y-1,new_pos_y+1):
				for k in range(new_pos_z-1,new_pos_z+1):
					new_grid[i][j][k]=sub_grid[new_pos_x-i][new_pos_y-j][new_pos_z-k]
		return new_grid
	
	def distribute_rotation(self,curr_ang,px,py,ang_to_rotate):
		
		
		seed_all=self.bel[px][py]
		new_all=np.zeros((self.odim),dtype='float')
		
		move_block=ang_to_rotate/self.discretization
		print "rotation move_block: "+str(move_block)
		
		for i in range(0,self.odim):
			new_all[(i+move_block)%self.odim]+=0.5*seed_all[i]
			new_all[(i+move_block+1)%self.odim]+=0.25*seed_all[i]
			new_all[(i+move_block-1)%self.odim]+=0.25*seed_all[i]
			
		self.bel[px][py]=new_all		
		#return new_grid
		
	def collect_all_possible_points(self):
		
		plist=[]
		for i in range(0,self.tdim):
			for j in range(0,self.tdim):
				for k in range(0,self.odim):
					if(any(self.bel[i][j][:]!=0)):
						p=Point()
						p.x=i
						p.y=j
						p.z=0
						plist.append(p)
		
		return plist
	
	def collect_every_possible_points(self):
		
		plist=[]
		for i in range(0,self.tdim):
			for j in range(0,self.tdim):
				for k in range(0,self.odim):
					if(self.bel[i][j][k]!=0):
						p=Point()
						p.x=i
						p.y=j
						p.z=k
						plist.append(p)
					
		return plist
	
	def scatter_val(self,new_grid,x,y,z,val):
		
		#new_mat=new_grid[x][y][z]*self.gkern2d(3,1)
		for k in range(0,self.odim):
			scal=new_grid[x][y][z]
			#print scal
			new_mat=scal*self.gkern2d(3,1)
			#Use x+2 instead of x+1
			#print "new_mat"
			#print new_mat
			
			#print "new_grid"
			#print x,y,k
			#print new_grid[x-1:x+2,y-1:y+2,k]
			new_grid[x-1:x+2,y-1:y+2,k]+=new_mat
			#print new_grid[x-1:y-1,x+2:y+2,k]
			#print new_grid[x-1:x+2,y-1:y+2,k]
		
		return new_grid
		
	
	def move_and_distribute_translation(self,dist,points):
		
		new_grid=np.zeros((self.tdim,self.tdim,self.odim),dtype='float')
		print "dist"+str(dist)
		for c in range(0,len(points)):
			p=Point()
			p.x=points[c].x
			p.y=points[c].y
			p.z=points[c].z
			
			curr_ang=p.z*self.discretization
			new_x=int(p.x+dist*np.cos(curr_ang))
			new_y=int(p.y+dist*np.sin(curr_ang))
			print "curr_ang"+str(curr_ang)
			val=self.bel[p.x][p.y][p.z]
			print "moving to "
			print new_x,new_y,p.z
			print "from"
			print p.x,p.y,p.z
			print "val"+str(val)
			new_grid=self.scatter_val(new_grid,new_x,new_y,p.z,val)
		
		
	
	def bayes_algorithm(self,msg,topic):
		
		print msg
		self.count+=1
		print "###############   COUNT   "+str(self.count)
		if(topic=='Observations'):
			#print "Observation msg received"
			
			r=msg.range
			tag_num=msg.tagNum
			bear=msg.bearing
			ang=self.get_angle_from_rotation(bear)
			#ang_inv=self.calc_inverse_angle_from_tag(ang,rng)
			points=self.collect_every_possible_points()
			#print self.tag.get(0)
			x_tag=self.tag.get(tag_num)[0]
			y_tag=self.tag.get(tag_num)[1]
			for i in range(0,len(points)):
				p=Point()
				dist=np.sqrt(pow((p.x-x_tag),2)+pow((p.y-y_tag),2))
				
				
				if(dist<1):
					scatter_val(p.x)
			
			
			
			'''
			val,curr_ang=self.get_curr_ang(self.px,self.py)
			curr_ang=curr_ang*self.discretization
			#curr_ang=val*self.discretization
			ang1=self.get_angle_from_rotation(msg.bearing)
			new_pos_x=self.px+int(rng*np.cos((ang1-curr_ang)*180))
			new_pos_y=self.py+int(rng*np.sin((ang1-curr_ang)*180))
			new_pos_z=self.pz+int(ang1/self.discretization)
			curr_val=self.bel[self.px,self.py,self.pz]
			dist_bel=self.distribute_new_pos(new_pos_x,new_pos_y,new_pos_z,curr_val)
			self.bel=np.add(self.bel,dist_bel)
			'''
		
		elif(topic=='Movements'):
			
			points=self.collect_all_possible_points()
			ang1=self.get_angle_from_rotation(msg.rotation1)
			#print "Euler angle"+str(ang1)
			#Distibute Rotation first
			for i in range(0,len(points)):
				#Angle to rotate
				p=points[i]
				curr_val,curr_pos=self.get_curr_ang(p.x,p.y)
				curr_ang=curr_pos*self.discretization
				
				self.distribute_rotation(curr_ang,p.x,p.y,ang1)
				'''
				for i in range(0,35):
					for j in range(0,35):
						for k in range(0,self.odim):
							if(self.bel[i][j][k]!=0):
								print i,j,k
								print self.bel[i][j][k]
				'''
			
			#Distribute Translation then for all points
			
			pts=self.collect_every_possible_points()
			#print "pts: "
			#print pts
			
			dist=msg.translation*5
			self.move_and_distribute_translation(dist,pts)
			
			#Distribute Rotation2 then
			points=self.collect_all_possible_points()
			ang2=self.get_angle_from_rotation(msg.rotation2)
			for i in range(0,len(points)):
				p=points[i]
				curr_val,curr_pos=self.get_curr_ang(p.x,p.y)
				curr_ang=curr_pos*self.discretization
				self.distribute_rotation(curr_ang,p.x,p.y,ang2)
			
			
			'''
			val,curr_ang=self.get_curr_ang(self.px,self.py)
			curr_ang=curr_ang*self.discretization
			#Get rotation1 first
			ang1=self.get_angle_from_rotation(msg.rotation1)
			#Get Translation 
			r=msg.translation  #Translation
			#Get rotation2 now
			final_ang=self.get_angle_from_rotation(msg.rotation2)
			
			
			pos_curr_ang_z=int(curr_ang/self.discretization)
			#Estimate new position now
			new_pos_x=int(r*np.cos((ang1-curr_ang)*180))
			new_pos_y=int(r*np.sin((ang1-curr_ang)*180))
			new_pos_z=int(final_ang/self.discretization)
			curr_val=self.bel[self.px,self.py,self.pz]
			dist_bel=self.distribute_new_pos(new_pos_x,new_pos_y,pos_curr_ang_z,curr_val)
			self.bel=np.add(self.bel,dist_bel)
			
			#Distribute new orientation
			
			self.distribute_new_pos(new_pos_x,new_pos_y,new_pos_z,1)
			self.update_pos()
			
			#self.bel=self.get_prob_action(x,u)*self.bel
			'''
		return self.bel
		
				
	def close_bag(self):
		self.bag.close()
	
	def publish_marker(self,pmsg):
		pmsg.header.frame_id = "base_link"
		pmsg.header.stamp = rospy.Time.now()
		pmsg.type = Marker.LINE_LIST
		pmsg.pose.orientation.w = 1.0
		pmsg.ns= "line_strip"
		pmsg.id = 1
		pmsg.scale.x = 0.1
		pmsg.color.a = 1.0
		pmsg.color.b = 1.0
		#for i in range(0,len(self.msg.points)):
		self.pub.publish(pmsg)
		
	
	def start(self):
		count=0
		points=[]
		self.bel[self.px][self.py][self.pz]=1
		#print np.max(self.bel)
		for topic, msg, t in self.bag.read_messages(topics=['Movements','Observations']):
			
			res=self.bayes_algorithm(msg,topic)
			i,j,k=self.get_estimated_pos(res)
			p=Point()
			p.x=i
			p.y=j
			p.z=k
			pmsg=Marker()
			pmsg.points.append(p)
			count+=1
			print "New position: <x,y,z>: <"+str(p.x)+","+str(p.y)+","+str(p.z)+">"
			if(count%10==0):
				#rospy.sleep(10)
				self.publish_marker(pmsg)
					
				#break	
		print "Final position: <x,y,z>: <"+str(p.x)+","+str(p.y)+","+str(p.z)+">"
		return 





if __name__=="__main__":
	fname='../bag/grid.bag'
	rospy.init_node('bfilter')
	px=12
	py=28
	pz=3
	bel=np.zeros((35,35,36),dtype='float')
	bel[px][py][pz]=1
	#Initial position needs to be 1
	#print bel
	obj=BayesianFil(fname,px,py,pz,bel)
	obj.start()
	obj.close_bag()

