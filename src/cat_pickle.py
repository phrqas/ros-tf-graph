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
#    Simple tool that reads a list of picke files and concatenates them
#    together.
#
#    Author: Pedro Santana
#    08/2013
#
import roslib; roslib.load_manifest('ros_tf_graph')
import argparse 
import pickle

def open_files(input_list):
    
    pickle_list=[]
    
    for file_name in input_list:    
        try:        
            pickle_list.append(open(file_name,"r"))                
        except IOError:            
            print "File "+file_name+" could not be opened for reading."
            return None            
    
    print "All files successfully opened."        
    return pickle_list
    

def write_pickle_file(objs,output_file):    
    try:        
        pickle.dump(objs,open(output_file,"w"),0)    
        return True    
    except IOError:                    
        return False    
    

def concatenate_files(input_descriptors):
    
    obj_list=[] #List of objects contained in all the files
    
    for pickle_file in input_descriptors:
        content = pickle.load(pickle_file)
        if(isinstance(content,list)):
            obj_list = obj_list+content
        else:
            obj_list = obj_list+[content]
    
    return obj_list

if __name__ == "__main__":    
                
    parser = argparse.ArgumentParser(description="Combines a list of objects from different pickle files into a single file.")
    parser.add_argument("-i","--input",dest="input_list",nargs="+",type=str, help="List of pickle files to be concatenated.")
    parser.add_argument("-o","--output",dest="output_file",type=str,help="Name of output pickle file.")
            
    args = parser.parse_args()
    
    input_descriptors = open_files(args.input_list)
    
    if(input_descriptors):
        obj_list = concatenate_files(input_descriptors)
        
        if(write_pickle_file(obj_list,args.output_file)):
            print "All objects successfully written to "+args.output_file
        else:
            print "Failed to write "+args.output_file
        
