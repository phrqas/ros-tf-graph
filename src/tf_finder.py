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
import roslib; roslib.load_manifest('ros_tf_graph')
import rospy; import tf
from tf.msg import tfMessage
from mers_srvs.srv import CoordinateTransform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf_tools import *
from mers_graph import *
import cat_pickle
import pickle
import argparse
import sys

global frame_graph, tf_listener, detected_pairs, validated_pairs
global det_period, det_count, record_to_file, backlash

        
def pack_transform(src_frame,dest_frame,trans,rot):
    transf = TransformStamped()    
    transf.header.frame_id = dest_frame
    transf.child_frame_id = src_frame
    transf.transform.translation = trans
    transf.transform.rotation = rot
    return transf
    
#Receives a request for a transform between two coordinate frames and
#replies 
def service_handler(req_msg):    

    #There should be a third parameter that tells us whether we want 
    #the estimate to be updated from scratch.
    #tf = is_visible(req_msg.frame_src,req_msg.frame_dest)
    
    #First tries to find the transform using the TF package
    #
    #    According to http://www.ros.org/wiki/tf/Overview/Transformations
    #    (trans,rot) = tf.TransformListener().lookupTransform("W", "A", now)
    #
    #    trans = translation of A wrt W
    #    rot = rotation of A wrt W
    #    
    invalid_frames = False
    if req_msg.frame_src == '':
        rospy.logwarn("Source frame is empty. Returning from the service.")
        invalid_frames = True

    if req_msg.frame_dest == '':
        rospy.logwarn("Destination frame is empty. Returning from the service.")
        invalid_frames = True

    if invalid_frames:
        rospy.logwarn("Invalid setting of coordinate frames.")
        return None

    try:
        rospy.loginfo("Using TF to lookup transform from "+req_msg.frame_src+" to "+req_msg.frame_dest)
        (trans,rot) = tf_listener.lookupTransform(req_msg.frame_dest,req_msg.frame_src,rospy.Time(0))
        
        #This is theoretically the wrong way to call TF
        #(trans,rot) = tf_listener.lookupTransform(req_msg.frame_src,req_msg.frame_dest,rospy.Time(0))
        tf_succeeded = True
    
    except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
        rospy.logwarn("TF couldn't find a transform from "+req_msg.frame_src+" to "+req_msg.frame_dest)
        tf_succeeded = False
    
    #If TF succeeded, it means that the two frames are part of the same tree
    if tf_succeeded:        
        rospy.loginfo("TF used to transform from "+req_msg.frame_src+" to "+req_msg.frame_dest)
        translation = Vector3(trans[0],trans[1],trans[2])
        rotation = Quaternion(rot[0],rot[1],rot[2],rot[3])            
        return pack_transform(req_msg.frame_src,req_msg.frame_dest,translation,rotation)    
    
    #If TF failed, either the frames are part of different, but connected, trees, or there is no connection between them.
    else:        
        
        #Used Breadth-First Search to find a path between the frames
        #with the smallest number of intermediate transforms
        searcher = BFSSearcher()            
        rospy.loginfo("Starting to search for a path from "+req_msg.frame_dest+" to "+req_msg.frame_src)        
        t_start = rospy.Time.now()        
        path= searcher.search(req_msg.frame_dest,req_msg.frame_src,frame_graph)        
        elapsed = rospy.Time.now()-t_start
        
        rospy.loginfo("Search took "+str(elapsed.to_nsec()/1e6)+" ms")
        
        if len(path)>0:
            print "Path found from "+req_msg.frame_dest+" to "+req_msg.frame_src
            searcher.print_path(path)
            
            if record_to_file:
                image_name = req_msg.frame_dest[1:]+"_to_"+req_msg.frame_src[1:]+".png"
                print "Writing file "+image_name
                frame_graph.to_image(image_name,path)
            
            tf_list = searcher.get_op_sequence(path,frame_graph)            
            
            if tf_list != None:
                rospy.loginfo("Computing final transform.")
                tf_final = compose_transform_list(tf_list)
                return pack_transform(req_msg.frame_src,req_msg.frame_dest,tf_final[0],tf_final[1])
            else:
                rospy.logerr("Error extracting sequence of transforms from the path.")
                return None
        
        else:
            print "No path found from "+req_msg.frame_dest+" to "+req_msg.frame_src
            return None
            
#    Valid coordinate frame pairs
#
def is_valid_pair(tf):
    
    parent_id = tf.header.frame_id
    child_id = tf.child_frame_id
    
    #Previosly validated pair
    if (parent_id,child_id) in validated_pairs:
        return True
        
    if detected_pairs.has_key((parent_id,child_id)):        
        current_values = detected_pairs[(parent_id,child_id)]
        
        #rospy.loginfo("Detected dictionary has key "+str((parent_id,child_id))+" with value "+str(current_values))
        
        if(rospy.get_time()-current_values[0]<=det_period):                        
            
            #rospy.loginfo("Pair "+str((parent_id,child_id))+" detected within 1s")
            
            if(current_values[1]>=det_count):
                #rospy.loginfo("Pair "+str((parent_id,child_id))+" is valid.")
                validated_pairs.append((parent_id,child_id))
                return True
            else:
                #rospy.loginfo("Updating count for "+str((parent_id,child_id)))
                detected_pairs[(parent_id,child_id)]= [rospy.get_time(),current_values[1]+1]
                return False
    
    #rospy.logwarn("Pair "+str((parent_id,child_id))+" first detected.")
    detected_pairs[(parent_id,child_id)]=[rospy.get_time(),1]
    return False
    
def add_to_visible(parent_id,child_id,tf_list):    
        
    #Discards empty strings
    if parent_id and child_id:        
        
        if frame_graph.add_edge_by_name(parent_id,child_id,tf_list,1):
            rospy.loginfo("New edge - "+parent_id+" -> "+child_id)
        
        #Adds reverse edge as well
        frame_graph.add_edge_by_name(child_id,parent_id,inverse_transform(tf_list),1)

# This function was added to fix a problem with how inconsistent frame
# names outputted by different ROS nodes. More specifically, when we
# upgraded ar_track_alvar to its most recent version (as of 09/13), it
# no longer appended a leading / to same frame and marker names.
def prepend_backlash(tf):
    if tf.header.frame_id[0]!='/':
        tf.header.frame_id = '/'+tf.header.frame_id
    if tf.child_frame_id[0]!='/':
        tf.child_frame_id = '/'+tf.child_frame_id

#Reads from the /tf topic and constantly updates the children of all
#published frames.
def tf_callback(tf_message):    
    
    transforms = tf_message.transforms

    for tf in transforms:    
        
        if backlash:
            prepend_backlash(tf)
        
        if(is_valid_pair(tf)):    
        
            parent_id = tf.header.frame_id
            child_id = tf.child_frame_id
        
            translation = tf.transform.translation
            rotation = tf.transform.rotation    
            
            #rospy.loginfo("Adding direct transform")
            add_to_visible(parent_id,child_id,[translation,rotation])
        

#Initializes the data structures and creates the frame_to_frame
#service.
if __name__ == "__main__":    
    
    rospy.init_node('tf_finder')            
        
    parser = argparse.ArgumentParser(description="Provides the /frame_to_frame service that computes transforms between arbitrary coordinate frames, provided that there is a path between them.")    
    parser.add_argument("-p","--period",dest="period",type=float, default=0.15, help="Period between successive detections of the same (parent,child) coordinate frame pair.")
    parser.add_argument("-c","--count",dest="count",type=int,default=5, help="Number of successive detections until a (parent,child) coordinate frame pair is considered valid.")
    parser.add_argument("-r","--record",dest="record",action="store_true", help="Tells tf_finder to write its outputs to files.")
    parser.add_argument("-l","--load-frame-graph",dest="graph_file",type=str, help="Pickle file containing a previously recorded frame graph.")
    parser.add_argument("-b","--backlash",dest="backlash",action="store_true", help="Tells tf_finder should prepend a backlash to all frame names.")
            
    args = parser.parse_args()
    
    if((args.period<=0.0) or(args.count<=0)):
        rospy.logerr("Period and count arguments should be positive numbers.")
    else:
        
        #Global parameters (this is really poor design, though...)
        det_period = args.period
        det_count = args.count
        record_to_file = args.record
        backlash = args.backlash
                                
        #Graph that stores the structure of tf frames from the /tf topic
        if args.graph_file:
            try:            
                rospy.loginfo("Loading graph from file "+args.graph_file)            
                frame_graph = pickle.load(open(args.graph_file,'r'))            
                rospy.loginfo("Graph successfully loaded.")                
            except IOError:    
                print "File "+args.graph_file+" could not be opened."
                sys.exit(0)
        else:
            #Empty graph                
            frame_graph = Graph()
        
        #Dictionary used to determine if a given pair of coordinate frames
        #has been consistently detected
        detected_pairs={}
        
        #List of validated frame pairs
        validated_pairs=[]
        
        tf_listener = tf.TransformListener() #Global TF listener        
        subs = rospy.Subscriber("/tf",tfMessage,tf_callback)    
        rospy.Service('frame_to_frame',CoordinateTransform,service_handler)
        
        # Sits idle waiting for service calls
        rospy.spin()    
        
        if(record_to_file):
            filename = "frame_graph.pickle"
            rospy.loginfo("Writing frame graph to pickle file "+filename)    
            cat_pickle.write_pickle_file(frame_graph,filename)
        
