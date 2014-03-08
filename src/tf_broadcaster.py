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
#
#
#
import roslib; roslib.load_manifest('ros_tf_graph')
import rospy; import tf
from ros_tf_graph.srv import CoordinateTransform
import cat_pickle
import argparse; import pickle

def read_tf_pairs(tf_file):
	
	tf_pairs=[]
	
	for line in tf_file:		
		tokens = line.split() #Splits line into tokens
		
		if(len(tokens)>0):	 #Line has tokens
			if(tokens[0][0] != '#'): #Not a comment
				if(len(tokens)==2):#Well-formed line with 2 frames.
					tf_pairs.append((tokens[0],tokens[1]))				
				else:					
					rospy.logwarn("Malformed file.")
					return []			
				
	return tf_pairs

def search_and_broadcast(frames_req,attempts,wait,br_freq,record_file):
	
	rospy.loginfo("Waiting for service 'frame_to_frame'. Did you initialize tf_finder?")
	rospy.wait_for_service('frame_to_frame')
	rospy.loginfo("Service 'frame_to_frame' found!")
	
	frame_srv = rospy.ServiceProxy('frame_to_frame',CoordinateTransform)
	tfs_found = [] #List of transforms found in the search
	frames_found = [] #List of frame pairs found in the search
		
	for trial in range(attempts):	
		for frame_pair in [pair for pair in frames_req if not pair in frames_found]:			
			
			src_frame = frame_pair[0]
			dest_frame = frame_pair[1]
			rospy.loginfo("Looking for transform from "+src_frame+" to "+dest_frame)			
			
			try:			
				response = frame_srv(frame_src=src_frame,frame_dest=dest_frame)
				
				if(response == None):
					rospy.logwarn("Service returned no transform from "+src_frame+" to "+dest_frame+" during attempt "+str(trial+1))
				else:
					rospy.loginfo("Found transform from "+src_frame+" to "+dest_frame+" during attempt "+str(trial+1))
					tfs_found.append(response.transform) #Stores the stamped transform object
					frames_found.append(frame_pair) #Stores the frame pair
					
			except (rospy.ServiceException):
				rospy.logwarn("Couldn't find a transform from "+src_frame+" to "+dest_frame+" during attempt "+str(trial+1))		
		
		if(len(frames_found)==len(frames_req)):
			rospy.loginfo("All transforms found at trial "+str(trial+1))
			break	#Stops looking for transforms
		else:		
			rospy.logwarn("Sleeping "+str(wait)+" before next attempt.")
			rospy.sleep(wait)	
			
	if(record_file):
		rospy.loginfo("Writing output to "+record_file)	
		cat_pickle.write_pickle_file(tfs_found,record_file)
	else:	
		broadcast_tf(tfs_found,br_freq)
	
		
def load_and_broadcast(load_file,br_freq):
	try:			
		rospy.loginfo("Loading file "+load_file)			
		tfs = pickle.load(open(load_file,'r'))		
		
		if(isinstance(tfs,list)):		
			broadcast_tf(tfs,br_freq)
		else:
			broadcast_tf([tfs],br_freq)
			
	except IOError:	
		print "File "+load_file+" could not be opened."


def broadcast_tf(tf_list,br_freq):
	
	tf_br = tf.TransformBroadcaster()	
	r = rospy.Rate(br_freq)	
	
	rospy.loginfo("Broadcasting all transforms with frequency "+str(br_freq)+" Hz")
	for transf in tf_list:
		print transf.header.frame_id+" <- "+transf.child_frame_id
		
	while not rospy.is_shutdown():		
		for transf in tf_list:	
			translation = (transf.transform.translation.x,transf.transform.translation.y,transf.transform.translation.z)
			rotation = (transf.transform.rotation.x,transf.transform.rotation.y,transf.transform.rotation.z,transf.transform.rotation.w)
			
			#	According to http://www.ros.org/wiki/tf/Overview/Transformations
			#
			#	tf.TransformBroadcaster().SendTransform(Rot,Trans,time,child,parent) 
			tf_br.sendTransform(translation,rotation,rospy.Time.now(),transf.child_frame_id,transf.header.frame_id)		
										
		r.sleep()

#Initializes the data structures and creates the frame_to_frame
#service.
if __name__ == "__main__":	
		
	parser = argparse.ArgumentParser(description="Broadcasts transforms to the ROS tf tree. Either uses tf_finder to discover new transforms or loads them from a pickle file.")	
	
	group = parser.add_mutually_exclusive_group()	
	group.add_argument("-f","--tf-file",dest="tf_file",type=str, help="File where desired transforms are listed.")
	group.add_argument("-l","--load-tf-file",dest="load_file",type=str, help="File containing a list of transforms that should be loaded.")		
		
	parser.add_argument("name",type=str, help="Node name (in case you would like to launch several).")
	parser.add_argument("-a","--attempts",dest="attempts",default=5,type=int, help="Maximum number of attempts before giving up.")
	parser.add_argument("-w","--wait",dest="wait",default=2.0,type=float, help="Number of seconds between attempts.")
	parser.add_argument("-b","--broadcast-frequency",dest="br_freq",default=15.0,type=float, help="TF broadcast frequency.")	
	parser.add_argument("-r","--record-on-file",dest="record_file",type=str, help="File where a list of transforms should be recorded.")
		
	args = parser.parse_args()
	
	rospy.init_node(args.name)	
		
	if(args.attempts<=0 or args.wait<0.0 or args.br_freq<=0.0):
		print "Invalid negative arguments."
	else:
		if(args.load_file):
			load_and_broadcast(args.load_file,args.br_freq)
		else:			
			try:			
				frames_req = read_tf_pairs(open(args.tf_file,'r'))	
				
				if(len(frames_req)>0):					
					search_and_broadcast(frames_req,args.attempts,args.wait,args.br_freq,args.record_file)												
				else:
					rospy.logerr("Each line in the configuration file should contain exactly two frame names.")
					
			except IOError:	
				print "File "+args.tf_file+" could not be opened."
			
		
	
	
	
	
	
			
	
	
	
		
		
