#!/usr/bin/env python
import roslib
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg
import rospy
import subprocess
import signal
import sensor_msgs
import actionlib
import rosbag
import tf
import numpy
import math
from std_msgs.msg import Int32, String
import sensor_msgs.point_cloud2 as pc2

import random;
from nav_msgs.msg import OccupancyGrid;

ranges=[]
angle_min = 0
angle_max = 0
angle_increment = 0
# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0
resolution=0.1
width=100
height=100

def create_map():
	global map_pub
	mapp= OccupancyGrid()
	mapp.info.resolution=resolution   ###new
	mapp.info.width=width        ###new
	mapp.info.height=height       #new
	mapp.info.origin.position.x=-5
	mapp.info.origin.position.y=-5
	mapp.info.origin.position.z=0
	mapp.info.origin.orientation.x=0
	mapp.info.origin.orientation.y=0
	mapp.info.origin.orientation.z=0
	mapp.info.origin.orientation.w=0
	mapp.data=[]
	for i in range (0,width*height):   ###new
		mapp.data.append(0.5)
	map_pub=rospy.Publisher('/map',OccupancyGrid, queue_size=0)
	map_pub.publish(mapp);
	return mapp

def distcalc(laserx,lasery,measx,measy):
  dist=math.sqrt(((measx-laserx)**2)+((measy+lasery)**2))  ###new
  #print(dist)
  return dist
  
def coordconverter(coord):
	#print(coord)
  	coord[0]= int(round(coord[0]*(1/mapp.info.resolution)))
	coord[1]= int(round(coord[1]*(1/mapp.info.resolution))) ###new

	coord[2]=0
  	#print(coord)
 	return coord

  
def log_inv_sensor_model(flaglast, distance , laser_range):
  #print(d,flaglast)
  if flaglast==1:
    #print("wall")
    if d*resolution < laser_range-0.25:
      #print("in range")
      # the sensor detects a wall for this cell
      #print("wall")
      return numpy.log(0.6 / (1 - 0.6))
      # the sensor detects free space for this cell
  return numpy.log(0.3 / (1 - 0.3))



#funcao de line rasterization
#input: vector 3d do laser e da medida
#output: array de coordenadas percorridas do laser ate a medida
def rasterize(origin,measurement):
	

	x1=origin[0]-0.5;
	y1=origin[1]-0.5;
	z1=0;
	#print('ggggggggggggggggggggggggggggggggggg')
	#print(measurement)
	x2=measurement[0]-0.5;
	y2=measurement[1]-0.5;
	z2=0;

	cell_array=[];
	yreff=y1;

	dx=int((x2)-(x1));
	dy=(y2)-(y1);

	if(dx==0):
		if(dy<0):
			m=1000
			b=-1000000
		elif(dy>0):
			m=-1000
			b=1000000
		else:
			coords = tuple([int(x1+0.5),int(y1+0.5),z1]);
			cell_array.append(coords);
			return cell_array;
	else:
		m=float(dy)/float(dx);
		b=y2-m*x2;


	if(x2>=x1):
		if(y2>y1):
			#quadrante 1;
			for c in range(0,dx+1):
				yr=m*(x1+c+0.5)+b;
				if yr>=y2:
					yr=y2;
				a=math.ceil(yr)
				d=int(yreff);
				vertical_cells=int(a-d);
				for i in range(1, vertical_cells+1):
					if (yreff>=y1 and yreff<=y2):

						coords = tuple([int(math.ceil(x1+c)),int(yreff+i),0]);
						cell_array.append(coords);
				yreff=yr;
		else:
			#quadrante 4;
			for c in range(0,dx+1):
				yr=m*(x1+c+0.5)+b;
				if yr<=y2:
					yr=y2;
				a=int(yr)
				d=math.ceil(yreff);
				vertical_cells=int(d-a);
				for i in range(0, vertical_cells):
					if (yreff<=y1 and yreff>=y2):
						coords = tuple([int(math.ceil(x1+c)),int(math.ceil(yreff-i)),0]);
						cell_array.append(coords);
				yreff=yr;
	else:
		if(y2>y1):
			for c in range(0,-dx+1):
				yr=m*(x1-c-0.5)+b;
				if yr>=y2:
					yr=y2;
				a=math.ceil(yr)
				d=int(yreff);
				vertical_cells=int(a-d);
				for i in range(1, vertical_cells+1):
					if (yreff>=y1 and yreff<=y2):
						coords = tuple([int(math.ceil(x1-c)),int(yreff+i),0]);
						cell_array.append(coords);
				yreff=yr;
		else:
			for c in range(0,-dx+1):
				yr=m*(x1-c-0.5)+b;
				if yr<=y2:
					yr=y2;
				a=int(yr)
				d=math.ceil(yreff);
				vertical_cells=int(d-a);
				for i in range(0, vertical_cells):
					if (yreff<=y1 and yreff>=y2):
						coords = tuple([int(math.ceil(x1-c)),int(math.ceil(yreff-i)),0]);
						cell_array.append(coords);
				yreff=yr;
	
	return cell_array


def callback(data):
	global valores
    	global ranges
	global angle_min
	global angle_max
	global angle_increment
	global max_range
	global min_range
	flag=0

    	max_range=data.range_max      ###new
   	min_range=data.range_min      ###new
	angle_min=data.angle_min
	angle_max=data.angle_max
	angle_increment=data.angle_increment
   	ranges=data.ranges
 
	#print(data.range_min, '...', data.range_max)
	if(flag==0):
		
		#print(angle_min,angle_max,angle_increment)
		flag=flag+1
	
    	

def position(data):
	global orientation
	global position
	pose=data.pose
	position=pose.position
	orientation=pose.orientation
	#print(position)

def calculate_coordenates(amin,amax,inc,values, point_vector):
	j=0
	alpha=amin
	#print('Size of vector:',len(values))    ##o values esta em q referencial? n e preciso multiplicar pela matriz?
	
	while alpha<=amax:
		
		#print(j,values[j])
		#x.append(1)
		#y.append(1)

		if values[j]>min_range and values[j]<max_range:     ###new
			
			x=values[j]*math.cos(alpha)
			
			y=values[j]*math.sin(alpha)

			point_vector=point_vector+[(x,y)]
		else:
			x=10000
			y=10000
		j+=1
		alpha=amin+inc*(j-1)
		#print(x[j],y[j])
		
	return point_vector

def calculate_position(position):
	print(position)
	return [position.x, position.y, position.z]

def transform(data):
	print(data)


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """

    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
#    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    #print(q)
    #return numpy.array([
    #    [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
    #    [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
    #    [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])
    return numpy.array([
        [1.0-2*q[2, 2]-2*q[1, 1],     2*q[0, 1]-2*q[2, 3],     2*q[1, 3]+2*q[0, 2]],
        [    2*q[0, 1]+2*q[3, 2], 1.0-2*q[2, 2]-2*q[0, 0],     2*q[1, 2]-2*q[0, 3]],
        [    2*q[0, 2]-2*q[1, 3],     2*q[1, 2]+2*q[0, 3], 1.0-2*q[1, 1]-2*q[0, 0]]])


def laser_listener():
    	#rospy.init_node('laser_listener', anonymous=True)
    	rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,callback)
    	#rospy.spin()
	
def pose_listener():
	#rospy.init_node('pose_listener', anonymous=True)
	rospy.Subscriber("/vrpn_client_node/mocap_marker/pose",geometry_msgs.msg.PoseStamped,position)
    	#rospy.spin()

def listener():
	rospy.init_node('listener', anonymous=True)
	




if __name__ == '__main__':
	print("hello")
	global rot_matrixmm
	global rot_matrixmb
	global rot_matrixbl
	flag=0
	points=[[],[]]
	prior = (numpy.log(0.5 / (1 - 0.5)))
	listener()
	listener = tf.TransformListener()
	
	mapp=create_map()
	point_vector=[]
	laser_listener()
        mapa=[]
    	for i in range (0,width*height):   ###new
        	mapa.append(0) #NEW
		#mapa[i]=0
		
	#pose_listener()
	print("-starting-")
	while not rospy.is_shutdown():
		try:
			(transmm,rotmm) = listener.lookupTransform('map','mocap_marker',  rospy.Time(0))
			(transmb,rotmb) = listener.lookupTransform('base_link','mocap_marker',  rospy.Time(0))
			(transbl,rotbl) = listener.lookupTransform('base_laser_link','base_link', rospy.Time(0))

	    	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	   	 	continue
		
		
	    
	    	rot_matrixmm=quaternion_matrix(rotmm)
	    	rot_matrixmb=quaternion_matrix(rotmb)
	    	rot_matrixbl=quaternion_matrix(rotbl)
	


	    	#print('Map to Marker')
	    	#print(rot_matrixmm)
	    	#print(transmm)
	    	#print('Marker to Robot')
	    	#print(rot_matrixmb)
	    	#print(transmb)
	    	#print('Robot to Laser')
	    	#print(rot_matrixbl)
	    	#print(transbl)
		
		readinglaser=[0,0,0]
		readingmappoint=[0,0,0]
        	readingmaplaser=[0,0,0]
#If doesnt work use numpy.dot
        	

	    	if len(ranges)>1:     #??
	    		points=calculate_coordenates(angle_min,angle_max,angle_increment,ranges,point_vector)
			
			readingmaplaser=(numpy.dot(rot_matrixmm,(numpy.dot(rot_matrixmb,(numpy.dot(rot_matrixbl,numpy.transpose(readinglaser))+numpy.transpose(transbl))+numpy.transpose(transmb)))))+numpy.transpose(transmm)
			readingmaplaser=coordconverter(readingmaplaser)
			#print(len(points))
			for i in range(0,len(points)):
				readinglaser[0]=points[i][0]
				readinglaser[1]=points[i][1]
                
                
				if readinglaser[0]!=10000 and readinglaser[1]!=10000:  #new
					readingmappoint=(numpy.dot(rot_matrixmm,(numpy.dot(rot_matrixmb,(numpy.dot(rot_matrixbl,numpy.transpose(readinglaser))+numpy.transpose(transbl))+numpy.transpose(transmb)))))+numpy.transpose(transmm)
					#print(points[1])
					#print("From",readingmaplaser)
					#print("To",readingmappoint)
					          ###new
					readingmappoint=coordconverter(readingmappoint)          ###new
					raster_vector=rasterize(readingmaplaser,readingmappoint)
					#print('raster')
					#print(readingmaplaser,readingmappoint)
					#mapp.data[int(readingmaplaser[0]+readingmaplaser[1])]=0
					j=len(raster_vector)-1
					flag=0
					for n in range(len(raster_vector)):     ###new
			      
						if(n==j):                         ###new
							#print('hhhhhhhhhhhhhhhhh', raster_vector[i][1])
						    	#print(d)
					   		flag=1
		
						d=distcalc(readingmaplaser[0],readingmaplaser[1],raster_vector[n][0],raster_vector[n][1])     ###new
						#print(raster_vector)
						xl=raster_vector[n][0]+width/2
						yl=raster_vector[n][1]+height/2
						#print('[',xl,'][',yl,']')
						#print(prior)
						if xl>0 and yl>0 and xl<width and yl<height:
							mapa[xl+yl*mapp.info.width] = mapa[xl+yl*mapp.info.width] - prior + log_inv_sensor_model(flag,d,max_range) #NEW
							if mapa[xl+yl*mapp.info.width]>4.60517:
								mapa[xl+yl*mapp.info.width]=4.60517
						#print('valor',mapp.data[xl+yl])
						#map_pub.publish(mapp)  #NEW
						#flag=0

			# Apply the inverse transformation from log-odds to probability
		#for i in range(0, 1000000):
			#print("test")
		mapp.data = (1 - 1. / (1 + numpy.exp(mapa)))*100 #NEW
			#print(m)

		map_pub.publish(mapp)




