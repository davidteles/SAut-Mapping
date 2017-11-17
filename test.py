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


ranges=[]
angle_min = 0
angle_max = 0
angle_increment = 0
# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

flag=0;
bag = rosbag.Bag('lrf_mapping-dataset_with_mocap.bag')


def callback(data):
	global valores
    	global flag
    	global ranges
	global angle_min
	global angle_max
	global angle_increment

	angle_min=data.angle_min
	angle_max=data.angle_max
	angle_increment=data.angle_increment
   	ranges=data.ranges
	#print(data)
	if(flag==0):
		print(angle_min,angle_max,angle_increment)
	flag=flag+1;
	
    	

def position(data):
	print(data)

def calculate_coordenates(amin,amax,inc,values):
	j=0
	alpha=amin
	#print('Size of vector:',len(values))
	x=[]
	y=[]
	while alpha<=amax:
		
		#print(j,values[j])
		x.append(1)
		y.append(1)

		if values[j]>0 and values[j]<5:
			
			x[j]=values[j]*math.cos(alpha)
			
			y[j]=values[j]*math.sin(alpha)
		else:
			x[j]=10000
			y[j]=10000
		alpha=amin+inc*j
		#print(x[j],y[j])
		j+=1
		
	return [x,y]

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
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])


def laser_listener():
    	#rospy.init_node('laser_listener', anonymous=True)
    	rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,callback)
    	#rospy.spin()
	
def pose_listener():
	rospy.init_node('pose_listener', anonymous=True)
	rospy.Subscriber("/vrpn_client_node/mocap_marker/pose",geometry_msgs.msg.PoseStamped,position)
    	rospy.spin()

def listener():
	rospy.init_node('listener', anonymous=True)
	




if __name__ == '__main__':
	print("hello")
	points=[[],[]]
	listener()

	listener = tf.TransformListener()

	laser_listener()

	while not rospy.is_shutdown():
		try:
			(transmm,rotmm) = listener.lookupTransform('mocap_marker', 'map', rospy.Time(0))
			(transmb,rotmb) = listener.lookupTransform('base_link', 'mocap_marker', rospy.Time(0))
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
	    	#print(ranges)
	    	if len(ranges)>20:
	    		points=calculate_coordenates(angle_min,angle_max,angle_increment,ranges)

		readinglaser=[0,0,0]
		readingmap=[0,0,0]
	    	for i in range(0,len(points[1])):
			readinglaser[0]=points[0][i]
			readinglaser[1]=points[1][i]
			readingmap=rot_matrixbl*numpy.transpose(readinglaser)
			readingmaping=(rot_matrixmm*(rot_matrixmb*(rot_matrixbl*numpy.transpose(readinglaser)+numpy.transpose(transbl))+numpy.transpose(transmb)))+numpy.transpose(transmm)
			print(readingmap)

	
		

		


