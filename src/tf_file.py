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
#    Either reads a transform from the command line and writes it to a file
#    or prints the content of a transform file. 
#
#    Author: Pedro Santana
#    08/2013
#
import roslib; roslib.load_manifest('ros_tf_graph')
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import argparse 
import pickle

def load_and_print(print_file):
    try:            
        rospy.loginfo("Loading file "+print_file)            
        tfs = pickle.load(open(print_file,'r'))        
        
        if(not isinstance(tfs,list)):        
            tfs = [tfs]
                        
        for tf in tfs:
            print str(tf)
            
    except IOError:    
        print "File "+print_file+" could not be opened."


def write_tf_to_file(tf_file, trans, rot,parent_id,child_id):
    try:                        
        if(len(trans)!=3):
            rospy.logerr("Translation vector should have 3 components")
        else:
            if(len(rot)!=4):
                rospy.logerr("Quaternion should have 4 components")
            else:
                transf = TransformStamped()
                transf.transform.translation = Vector3(trans[0],trans[1],trans[2])
                transf.transform.rotation = Quaternion(rot[0],rot[1],rot[2],rot[3])
                transf.child_frame_id = child_id
                transf.header.frame_id = parent_id
                
                pickle.dump(transf,open(tf_file,"w"),0)        
                print "TF successfully written to "+tf_file
    except IOError:            
        print "File "+tf_file+" could not be opened for writing."
    
    
if __name__ == "__main__":    
                
    parser = argparse.ArgumentParser(description="Writes a stamped transform to a pickle file or prints its content.")
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-f","--tf-file",dest="tf_file",type=str, help="File where desired transform should be written.")
    group.add_argument("-p","--print-tf-file",dest="print_file",type=str, help="File containing a list of transforms that should be printed.")
    
    parser.add_argument("-t","--translation",dest="translation",nargs="+",type=float, help="List representing (X,Y,Z) translation.")
    parser.add_argument("-r","--rotation",dest="rotation",nargs="+",type=float, help="List representing (X,Y,Z,W) quaternion.")
    parser.add_argument("-e","--parent",dest="parent",type=str, help="Frame id (parent).")        
    parser.add_argument("-c","--child",dest="child",type=str, help="Child id (child).")        
    
    args = parser.parse_args();
    
    if(args.print_file):
        load_and_print(args.print_file)
    else:
        write_tf_to_file(args.tf_file, args.translation, args.rotation,args.parent,args.child)
    
        
