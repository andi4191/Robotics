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
		
		self.bag=bag
		self.px=px
		self.py=py
		self.pz=pz
		self.bel=bel
		self.sigma_sensor=2
		self.sigma_action=2
		self.tdim=35
		self.dim=3
		self.odim=4
		self.discretization=np.pi/2
		self.pub=rospy.Publisher('visualization_marker',Marker, queue_size=100)

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
		#temp=[]
		max_val=-999
		#np.array((self.dim*self.odim))
		'''
		for r in range(i-int(self.dim/2),i+int(self.dim)):
			for c in range(j-int(self.dim/2),j+int(self.dim/2)):
				for z in range(0,self.odim):
					if(self.bel[r][c][z]>max)
				temp.append(self.bel[r][c][:])
		
		'''
		pos=0
		for k in range(0,self.odim):
			if(self.bel[i][j][k]>max_val):
				max_val=self.bel[i][j][k]
				pos=k
		
		#a=np.array(temp,dtype='float')
		
		#print "returning "+str(a)+" : "+str(np.max(a))
		#return np.max(a)
		
		return max_val,pos
				
	def gkern2(self,kernlen, nsig):
		"""Returns a 2D Gaussian kernel array."""
		# create nxn zeros
		inp = np.zeros((kernlen, kernlen,kernlen))
		# set element at the middle to one, a dirac delta
		inp[kernlen//2, kernlen//2, kernlen//2] = 1
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
		print sub_grid
		print new_pos_x,new_pos_y,new_pos_z
		for i in range(new_pos_x-1,new_pos_x+1):
			for j in range(new_pos_y-1,new_pos_y+1):
				for k in range(new_pos_z-1,new_pos_z+1):
					new_grid[i][j][k]=sub_grid[new_pos_x-i][new_pos_y-j][new_pos_z-k]
		return new_grid
		
	def bayes_algorithm(self,msg,topic):
		
		
		if(topic=='Observations'):
			#print "Observation msg received"
			print msg
			rng=msg.range
			tag_num=msg.tagNum
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
			#pass
			#print "Anurag"
			'''
			self.bel=self.get_prob_given(z,x)*self.bel
			#Normalize all
			self.bel=self.bel/(np.sum(self.bel))
			'''
		
		elif(topic=='Movements'):
			print msg
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
		for topic, msg, t in self.bag.read_messages(topics=['Movements','Observations']):
			#print topic
			'''
			if(topic=='Movements'):
				self.motion_model(msg)
				#pass
			else:  #Observations
				self.sensor_model(msg)
				#pass
			'''
			#print "Result: "
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
	bel=np.zeros((35,35,4),dtype='float')
	bel[px][py][pz]=1
	#Initial position needs to be 1
	#print bel
	obj=BayesianFil(fname,px,py,pz,bel)
	obj.start()
	obj.close_bag()

