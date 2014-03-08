ros-tf-graph
============

Extension of the ROS tf graph that allows calibration between sensors using triangulation.

# Quick overview

* tf_finder: listens to the messages in the /tf topic and records a graph of all received transforms. Also provides a service that finds transforms between frames (including triangulated ones) using Breadth-First Serch or says that no such transforms exists. Quick help is available with the -h option. You might want to invoke it like this

    rosrun ros_tf_graph tf_finder.py -b

* tf_broadcaster: serves two purposes: I) Reads a configuration file containing a list of desired transforms (see sensor_frames.txt) and polls tf_finder to get the transforms. Once it's done, writes a pickle file with all transforms found. II) loads a pickle file containing transforms and broadcasts them.

  * Polling tf_finder: 
    
    rosrun ros_tf_graph tf_broadcaster.py b1 -f sensor_frames_file -r calibration_file

  * Broadcasting transforms: 

    rosrun ros_tf_graph tf_broadcaster.py b1 -l <path to where the calibration file was recorded>

# Troubleshooting:

* Make sure to give tf_finder enough time to build up a transform graph from your sensors (Baxter cameras, Kinects, Alvar, etc.) before you start polling it. A fraction of a second should be sufficient, so just make sure to launch tf_finder before you use tf_broadcaster.

* tf_finder depends on pydot to generate visualizations of the search graphs. It isn't required, but I do find it helpful to know what is going on in terms of the search being performed and the triangulations. 
