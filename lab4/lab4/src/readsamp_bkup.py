#!/usr/bin/python
from __future__ import division
import rosbag
import sys
import rospy
import tf
from lab4.msg import Motion
import scipy.ndimage.filters as fi
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
np.set_printoptions(threshold='nan')
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
		self.odim=36
		self.discretization=np.pi/18
		self.kern2=self.gkern2(3,1)
		self.tag={0:[6,26],1:[6,16],2:[6,6], 3:[21,6], 4:[21,16], 5:[21,26]}
		self.pub=rospy.Publisher('visualization_marker',Marker, queue_size=100)
		self.points=[]
		#self.pub=rospy.Publisher('tag_marker',Marker, queue_size=100)
		#self.path_pub=rospy.Publisher('path_marker',Marker, queue_size=100)

	def get_angle_from_rotation(self,rotation):
		quaternion = (rotation.x,rotation.y,rotation.z,rotation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw=euler[2]
		
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
			
	def get_coordinate_for_radian(self,val):
		
		ret_val=int(np.round((val/self.discretization)))%self.odim
		#int(np.fabs(val//self.discretization))
		return ret_val
	
	def get_tag_pos(self,tag_num):
		
		p=Point()
		tmp=self.tag.get(tag_num)
		p.x=tmp[0]
		p.y=tmp[1]
		p.z=0
		return p
	
	def get_curr_ang(self,i,j):
		
		max_val=-999
		pos=0
		for k in range(0,self.odim):
			if(self.bel[i][j][k]>max_val):
				max_val=self.bel[i][j][k]
				pos=k
		
		return max_val,pos
				
	def gkern2(self,klen, nsig):
		
		inp = np.zeros((klen, klen))
		inp[klen//2, klen//2] = 1
		
		return fi.gaussian_filter(inp, nsig)
	
	def gkern3(self,klen, nsig):
		inp = np.zeros((klen, klen, klen))
		inp[klen//2, klen//2, klen//2] = 1
		return fi.gaussian_filter(inp, nsig)
	
	
	def copy_mat(self,new_grid,mat,x,y,z):
		num_row=len(mat)
		num_col=len(mat[0])
		#print int(x),(num_row//2)
		x=int(x)
		y=int(y)
		z=int(z)
		
		impact_pts=[]
		
		for ix,i in enumerate(range((x-int(num_row//2)),(x+int(num_row//2)+1)%self.tdim)):
			for jy,j in enumerate(range((y-int(num_col//2)),(y+int(num_col//2)+1)%self.tdim)):
				#print i,j,ix,jy
				pt=Point()
				pt.x=i
				pt.y=j
				pt.z=z
				new_grid[i][j][z]=mat[ix][jy]
				impact_pts.append(pt)
		
		return new_grid,impact_pts
	
	def copy_mat3(self,new_grid,mat,p,dim):
		
		r=c=d=dim
		
		x=int(p.x)
		y=int(p.y)
		z=int(p.z)
		
		#print "for point",x,y,z
		'''
		for i,ix in enumerate(range(x-(r//2),x+(r//2)+1)):
			for j,jy in enumerate(range(y-(c//2),y+(c//2)+1)):
				for k,kz in enumerate(range(z-(d//2),z+(d//2)+1)):
					if(ix==self.tdim or jy==self.tdim or i==dim or j==dim):
						continue
					
					new_grid[ix][jy][kz%self.odim]=mat[i%dim][j%dim][k%dim]
		'''
		
		
		for i,ix in enumerate(range(x-(r//2),(x+(r//2))%self.tdim+1)):
			for j,jy in enumerate(range(y-(c//2),(y+(c//2))%self.tdim+1)):
				for k,kz in enumerate(range(z-(d//2),(z+(d//2))%self.odim+1)):
					#print ix,jy,kz,"--",i,j,k
					if(ix==self.tdim or jy==self.tdim or i==dim or j==dim):
						continue
					
					new_grid[ix][jy][kz]=mat[i%dim][j%dim][k%dim]
		
		'''
		for i,ix in enumerate(range(x-(r//2),x+((r//2)%self.tdim)+1)):
			for j,jy in enumerate(range(y-(c//2),y+((c//2)%self.tdim)+1)):
				for k,kz in enumerate(range(z-(d//2),z+((d//2)%self.odim)+1)):
					new_grid[ix][jy][kz]=mat[i][j][k]
		'''
		
		return new_grid
	
	def distribute_new_pose(self,points,new_point,FLAG,prev_val_list):
		
		
		new_grid=np.zeros((self.tdim,self.tdim,self.odim),dtype='float')
		
		#To prevent from updating the main bel grid
		prev_grid=np.copy(self.bel)
		xy_kern=self.kern2  #self.gkern2(3,1)
		kern3=self.gkern3(5,1)
		if(FLAG=="TRANSROTATION"):
			for i in range(0,len(points)):
				p=points[i]
				new_p=new_point[i]
				
				prev_val=prev_val_list[i]
				prev_grid[p.x][p.y][p.z]=0
				mat=prev_val*kern3
				
				tmp_mat=self.copy_mat3(new_grid,mat,new_p,3)
				new_grid=np.add(prev_grid,tmp_mat)
				
		
		else:			
			
						
			for i in range(0,len(points)):
				
				p=points[i]
				new_p=new_point[i]
				
				prev_val=prev_val_list[i]
				prev_grid[p.x][p.y][p.z]=0
				move_blocks=new_p.z-p.z
				
				new_grid=self.distribute_new_orien(new_grid,move_blocks,new_p,prev_val) 
					
		
		
		self.bel=np.copy(new_grid)
		self.update_pos()
		
	
	def rad_to_deg(self,val):
		return (180/np.pi)*val
	
	def calc_dist(self,x1,y1,x2,y2):
		
		ret_val=np.sqrt(pow((x1-x2),2)+pow((y1-y2),2))
		return ret_val		
	
	def update_pos_mov(self,points,ang,dist):
		
		FLAG=""
		if(dist==0.0):
			FLAG="ROTATION2"
		else:
			FLAG="TRANSROTATION"
		
		
		nw_point=[]
		pst_val=[]
		for i in range(0,len(points)):
			prev_p=Point()
			p=Point()
			prev_p.x=points[i].x
			prev_p.y=points[i].y
			prev_p.z=points[i].z
			pst_val.append(self.bel[prev_p.x][prev_p.y][prev_p.z])
			self.bel[prev_p.x][prev_p.y][prev_p.z]=0
			temp=[]
			
			if(FLAG=="TRANSROTATION"):
				
				#p.x=int(points[i].x+(dist*np.cos(ang)))
				#p.y=int(points[i].y+(dist*np.sin(ang)))
				p.x=points[i].x+int(dist*np.cos(ang))
				p.y=points[i].y+int(dist*np.sin(ang))
				
				#Current angle should be considered as well
			
				tmp=self.bel[prev_p.x][prev_p.y][:].tolist()
				idx=tmp.index(np.max(tmp))
			
				indx=self.get_coordinate_for_radian(ang)
				p.z=indx
				
				#print "p <",points[i].x,points[i].y,points[i].z,"> new_point <",p.x,p.y,p.z,">"
				
				nw_point.append(p)
			
			elif(FLAG=="ROTATION2"):
				
				pt=Point()
				
				pt.x=prev_p.x
				pt.y=prev_p.y
				idx=prev_p.z
				
				indx=self.get_coordinate_for_radian(ang)
				pt.z=indx
				
				nw_point.append(pt)
		
		self.distribute_new_pose(points,nw_point,FLAG,pst_val)
		
	
	def inflict_vicinity(self,point):
		dim=5
		sig=1
		kern3=self.gkern3(dim,sig)
		
		#Need to copy the kernel with prob 1 distributed around center point
		
		prev_grid=np.copy(self.bel)
		new_grid=np.zeros((self.tdim,self.tdim,self.odim),dtype='float')
		tmp_mat=self.copy_mat3(new_grid,kern3,point,dim)
		new_grid=np.add(prev_grid,tmp_mat)
		
		
		self.bel=np.copy(new_grid)
		self.update_pos()
		
		
		
	
	def update_pos_obs(self, new_pos,theta):
		
		
		new_grid=np.zeros((self.tdim,self.tdim,self.odim),dtype='float')
		#To prevent from updating the main bel grid
		prev_grid=np.copy(self.bel)
		
		#Need to distribute the probability of 1 in the area near the expected position of bot
		flag=False
		if(new_pos.x<0 or new_pos.x>self.tdim or new_pos.y<0 or new_pos.y>self.tdim):
			flag=True
		
		if(False==flag):
			print "inflict the prob near new_pos <",new_pos.x,new_pos.y,new_pos.z,">"
			self.inflict_vicinity(new_pos)		
		
			
	
	
	def distribute_new_orien(self,new_grid,move_blocks,point,val):
		
		new_z=point.z
		
		#if(val>0.001):
		if(val!=0):
				
			new_grid[point.x][point.y][new_z]+=val*0.40780719
			new_grid[point.x][point.y][(new_z-1)%self.odim]+=val*0.2960964
			new_grid[point.x][point.y][(new_z+1)%self.odim]+=val*0.2960964
		
		return new_grid
		
				
		
	def update_pos(self):
		
		idx=kdx=jdx=0;
		max_val=-99999
		for i in range(0,self.tdim):
			for j in range(0,self.tdim):
				for k in range(0,self.odim):
					
					if(self.bel[i][j][k]>max_val):
						max_val=self.bel[i][j][k]
						
						idx=i
						jdx=j
						kdx=k
		self.px=idx
		self.py=jdx
		self.pz=kdx			
	
	
	def collect_non_zero_points(self,grid):
		
		threshold=0.005
		points=[]
		for i in range(0,self.tdim):
			for j in range(0,self.tdim):
				for k in range(0,self.odim):
					
					if(grid[i][j][k]>threshold):
						p=Point()
						p.x=i
						p.y=j
						p.z=k
						
						points.append(p)
						#print "NZ <",p.x,p.y,p.z,">: ",grid[p.x][p.y][p.z]
		
		return points
		
	def get_my_orien(self):
		
		ret_val=self.pz*(2*np.pi/self.odim)
		return  ret_val
		
	
	def handlemsg(self,msg,topic):
		
		print msg.timeTag
		if(msg.timeTag==17):
			print "verify curr_pos ",self.px,self.py,self.pz,"value ",self.bel[self.px][self.py][self.pz]
		
		if(topic=='Observations'):
			
			r=msg.range
			r=(r*100)/20  
			
			tag_num=msg.tagNum
			#val,curr_ang=self.get_curr_ang(self.px,self.py)
			
			theta=self.get_my_orien()
			curr_ang=self.get_coordinate_for_radian(theta)
						
			#theta=curr_ang=curr_ang*self.discretization
			
			
			beta=ang1=self.get_angle_from_rotation(msg.bearing)
			print "theta,beta,r ",theta,beta,r," in deg <",self.rad_to_deg(theta),self.rad_to_deg(beta),">"
			
			tag_pos=self.get_tag_pos(tag_num)
			
			new_pos=Point()
			new_pos.x=int(tag_pos.x-r*np.cos(theta+beta))
			new_pos.y=int(tag_pos.y-r*np.sin(theta+beta))
			print "curr_pos ",self.px,self.py,self.pz," tag_pos ",tag_pos.x,tag_pos.y,"theta+beta ",theta+beta
			#new_pos.x=tag_pos.x-int(r*np.cos(theta+beta))
			#new_pos.y=tag_pos.y-int(r*np.sin(theta+beta))
			new_pos.z=self.get_coordinate_for_radian(theta+beta)
			##print "theta",theta,new_pos.z
			points=self.collect_non_zero_points(self.bel)
			#upd_points=[]
			min_d=9999
			sol_point=Point()
			for i in range(0,len(points)):
				d=self.calc_dist(points[i].x,points[i].y,tag_pos.x,tag_pos.y)
				
				if(d<min_d):
					min_d=d
					sol_point.x=points[i].x
					sol_point.y=points[i].y
					min_d_ang=999
					for th in range(0,self.odim):
						th_delta=new_pos.z-th
						if(abs(th_delta)<min_d_ang):
							min_d_ang=th_delta
							sol_point.z=th
			print "sol_point",sol_point.x,sol_point.y,sol_point.z,min_d
			for i in range(0,len(points)):
				if(sol_point.x==points[i].x or sol_point.y==points[i].y or sol_point.z==points[i].z):
					continue
				else:
					self.bel[points[i].x][points[i].y][points[i].z]=0
			#upd_points.append(sol_point)
			print "max_val_check ",np.max(self.bel)
			self.update_pos_obs(sol_point,theta)
			print "#Obs curr_pos <",self.px,self.py,self.pz,">"
			
		elif(topic=='Movements'):
			
			
			my_ang=self.get_my_orien()
			
			#Get rotation1 first
			ang1=self.get_angle_from_rotation(msg.rotation1)
			a1=ang1
			ang1+=my_ang
			
			#Get rotation2 
			ang2=self.get_angle_from_rotation(msg.rotation2)
			a2=ang2
			ang2+=my_ang
			
			
			#We Get translation in meters
			dist=msg.translation
			dist=(dist*100)/20   #Change it into grid cells
			
			#dist=int(dist)
			
			#Combining the rotation1 and translation for optimization
			
			points=self.collect_non_zero_points(self.bel)
			p1=Point()
			p1.x=self.px
			p1.y=self.py
			p1.z=self.pz
			print "before up curr_pos,",self.px,self.py,self.pz,"ang,translate,ang <",self.rad_to_deg(ang1),dist,self.rad_to_deg(ang2),"> max_value ",np.max(self.bel)," at curr_pos ",self.bel[self.px][self.py][self.pz]
			#if(msg.timeTag==17):
			#	print self.bel[6:9][14:19][:]
			self.update_pos_mov(points,ang1,dist)
			
			print "curr_pos,",self.px,self.py,self.pz,"ang,translate,ang <",self.rad_to_deg(ang1),dist,self.rad_to_deg(ang2),"> max_value ",np.max(self.bel)," at curr pos ",self.bel[self.px][self.py][self.pz]
			print "prev_anticipated pt val",self.bel[p1.x][p1.y][p1.z]
			
			
			#print "Turn for Rotation_2"
			points=self.collect_non_zero_points(self.bel)
			
			self.update_pos_mov(points,ang2,0)
		#self.update_pos()
		print "updated_pos ",self.px,self.py,self.pz
		return self.bel
		
				
	def close_bag(self):
		self.bag.close()
	
	def publish_tags(self):
		
		marker = Marker()
		marker.header.frame_id="base_laser_link"
		marker.header.stamp = rospy.Time.now()
			
		marker.id=2
		marker.action=Marker.ADD
		marker.type = Marker.POINTS
		marker.ns="tags"
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		
		marker.color.g = 1.0
		
		marker.color.a = 1.0
			
		
		for i in range(0,6):
			
			
			tmp=self.get_tag_pos(i)
			p=Point()
			p.x=tmp.x
			p.y=tmp.y
			p.z=0
			#print "point ",p.x,p.y,p.z
			marker.points.append(p)
		self.pub.publish(marker)
			
					
	
	def publish_marker(self):
		
		
		self.publish_tags()
		pmsg=Marker()
		pmsg.header.frame_id = "base_laser_link"
		pmsg.header.stamp = rospy.Time.now()
		pmsg.type = Marker.LINE_LIST
		pmsg.action=Marker.ADD
		#print "val of ADD",Marker.ADD
		
		#pmsg.pose.orientation.w = 1.0
		pmsg.ns= "line_strip"
		pmsg.id = 3
		pmsg.scale.x = 0.2
		#pmsg.scale.y = 0.4
		#pmsg.scale.z = 0.4
		pmsg.color.a = 1.0
		pmsg.color.b = 1.0
		print "num of points ",len(self.points)
		for i in range(0,len(self.points)-1):
			p=Point()
			p.x=self.points[i].x
			p.y=self.points[i].y
			p.z=0
			
			j=i+1
			pt=Point()
			pt.x=self.points[j].x
			pt.y=self.points[j].y
			pt.z=0
			
			pmsg.points.append(p)
			#d=self.calc_dist(p.x,p.y,pt.x,pt.y)
			
			pmsg.points.append(pt)
			
			#print d
			'''
			if(d>1):
				print d
				print p.x,p.y,p.z," --> ",pt.x,pt.y,pt.z
			'''
			#print "plotting points ",p.x,p.y,p.z
		
		self.pub.publish(pmsg)
		
		
	
	def start(self):
		count=0
		points=[]
		
		
		for topic, msg, t in self.bag.read_messages(topics=['Movements','Observations']):
			
			print msg
			res=self.handlemsg(msg,topic)
			p=Point()
			p.x=self.px
			p.y=self.py
			p.z=self.pz
			
			self.points.append(p)
			print "Appending <x,y,z>: <"+str(p.x)+","+str(p.y)+","+str(p.z)+"> count: ",count		
			
			self.publish_marker()
			
			if(count>25):
				#break
				pass
				
			count+=1
			
		print "Final position: <x,y,z>: <"+str(p.x)+","+str(p.y)+","+str(p.z)+">"
		#print self.points
		
		
		return 





if __name__=="__main__":
	
	fname=sys.argv[1]
	rospy.init_node('bfilter')
	px=12
	py=28
	pz=20
	
	bel=np.zeros((35,35,36),dtype='float')
	
	bel[px][py][pz]=1
	#Initial position needs to be 1
	obj=BayesianFil(fname,px,py,pz,bel)
	obj.start()
	obj.close_bag()
	
	

