#!/usr/bin/env python
#
#Copyright (c) 2014 Pedro Santana
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

import roslib; roslib.load_manifest('ros_tf_graph')
import numpy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import math

#Takes a transform
def inverse_transform(tf_A_wrt_W):    
    q_W_wrt_A = quaternion_conjugate(tf_A_wrt_W[1])    
    t_W_wrt_A = quaternion_rotate(q_W_wrt_A,vector_neg(tf_A_wrt_W[0]))
    return [t_W_wrt_A,q_W_wrt_A]

#Composes the two transforms
def compose_transforms(tf_A_wrt_W,tf_P_wrt_A):
    t1 = tf_A_wrt_W[0]
    q1 = tf_A_wrt_W[1]
    
    t2 = tf_P_wrt_A[0]
    q2 = tf_P_wrt_A[1]
    
    q = quaternion_product(q1,q2)
    t = vector_add(quaternion_rotate(q1,t2),t1)
    
    return [t,q]

def compose_transform_list(tf_list):    
    if len(tf_list)>0:
        tf_comp = tf_list[0]
    
        for i in range(len(tf_list)-1):
            tf_comp = compose_transforms(tf_comp,tf_list[i+1])
        
        return tf_comp
    else:
        return []


def quaternion_to_axis_angle(q):
    
    angle_rad = 2*math.acos(q.w)        
    sin = math.sin(angle_rad/2.0)
    
    if sin != 0.0:        
        axis_un = Vector3(q.x/sin,q.y/sin,q.z/sin)    
        norm = vector_norm(axis_un)    
        axis = Vector3(axis_un.x/norm,axis_un.y/norm,axis_un.z/norm)
    else:
        axis = Vector3(1,0,0)
        
    return [axis, angle_rad]

def quaternion_to_matrix(q):
    
    a = q.w; b = q.x; c=q.y; d=q.z 
    
    m = numpy.zeros([3,3],dtype=numpy.float64)
    
    m[0,0] = a**2+b**2-c**2-d**2
    m[1,1] = a**2-b**2+c**2-d**2
    m[2,2] = a**2-b**2-c**2+d**2    
    m[0,1] = 2*(b*c-a*d)
    m[1,0] = 2*(b*c+a*d)    
    m[0,2] = 2*(b*d+a*c)
    m[2,0] = 2*(b*d-a*c)    
    m[1,2] = 2*(c*d-a*b)
    m[2,1] = 2*(c*d+a*b)
    
    return m
    
    

def quaternion_sqnorm(q):
    return q.x**2+q.y**2+q.z**2+q.w**2
    

def quaternion_product(q1,q2):
    
    q_prod = Quaternion()
        
    a1= q1.w; b1= q1.x; c1= q1.y; d1= q1.z
    a2= q2.w; b2= q2.x; c2= q2.y; d2= q2.z
    
    q_prod.w = a1*a2 - b1*b2 - c1*c2 - d1*d2
    q_prod.x = a1*b2 + b1*a2 + c1*d2 - d1*c2
    q_prod.y = a1*c2 - b1*d2 + c1*a2 + d1*b2
    q_prod.z = a1*d2 + b1*c2 - c1*b2 + d1*a2
    
    #This is exactly the same quaternion as before, but with positive
    #real part (seems to be the standard for ROS TF)
    if q_prod.w < 0.0:
        q_prod.x = -q_prod.x
        q_prod.y = -q_prod.y
        q_prod.z = -q_prod.z
        q_prod.w = -q_prod.w
    
    return q_prod



def quaternion_conjugate(q):
    
    q_conj = Quaternion()
        
    q_conj.x = -q.x
    q_conj.y = -q.y
    q_conj.z = -q.z
    q_conj.w = q.w
    
    return q_conj

#Rotates a vector by a quaternion
def quaternion_rotate(q,v):

    #~ v_q = Quaternion(v.x,v.y,v.z,0) #Purely imaginary quaternion    
    #~ q_conj = quaternion_conjugate(q)
#~ 
    #~ #v_q_rot = q*v_q*q_conj
    #~ v_q = quaternion_product(q,v_q)
    #~ v_q = quaternion_product(v_q,q_conj)    
    
    m = quaternion_to_matrix(q)
    v_ = numpy.array([v.x,v.y,v.z])
    
    v_rot = m.dot(v_)
    
    return Vector3(v_rot[0],v_rot[1],v_rot[2])
    
    #~ if(v_q.w>1e-10):
        #~ rospy.logerr("Potential error rotating vector by a quaternion. Real part is "+str(v_q.w))
    #~ 
    #~ return Vector3(v_q.x,v_q.y,v_q.z)
    

def vector_norm(v):
    return math.sqrt(v.x**2+v.y**2+v.z**2)
    

def vector_add(v1,v2):
    
    add = Vector3()
    
    add.x = v1.x+v2.x
    add.y = v1.y+v2.y
    add.z = v1.z+v2.z
    
    return add

def vector_diff(v1,v2):
    
    diff = Vector3()
    
    diff.x = v1.x-v2.x
    diff.y = v1.y-v2.y
    diff.z = v1.z-v2.z
    
    return diff

def vector_neg(v):
    return Vector3(-v.x,-v.y,-v.z)
