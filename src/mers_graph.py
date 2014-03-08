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

import sys
import pydot


class Graph():
    
    def __init__(self):
        self.node_dict = {}        
        self.neighbor_dict={}
    
    def add_node(self,graph_node):        
        self.node_dict[graph_node.name] = graph_node                     
    
    def add_edge_and_nodes(self,parent,child,edge_op,edge_weight):        
        new_edge = False
    
        if not parent in self.node_dict.values():
            self.add_node(parent)
            new_edge = True
        if not child in self.node_dict.values():
            self.add_node(child)
            new_edge = True
        
        self._add_edge(parent,child,edge_op,edge_weight)            
        
        return new_edge
    
    def add_edge_by_name(self,parent_name,child_name,edge_op,edge_weight):
        
        new_edge = False
        
        if not parent_name in self.node_dict.keys():            
            self.add_node(GraphNode(parent_name))
            new_edge = True
        if not child_name in self.node_dict.keys():
            self.add_node(GraphNode(child_name))
            new_edge = True
        
        parent = self.node_dict[parent_name]
        child = self.node_dict[child_name]
            
        self._add_edge(parent,child,edge_op,edge_weight)
        
        return new_edge
    
    def _add_edge(self,parent,child,edge_op,edge_weight):
        if self.neighbor_dict.has_key(parent):
            neighbors = self.neighbor_dict[parent]
            neighbors[child] = [edge_weight,edge_op]
        else:
            self.neighbor_dict[parent]={child:[edge_weight,edge_op]}        
        
    def get_node(self,node_name):
        if self.node_dict.has_key(node_name):
            return self.node_dict[node_name]
        else:
            return None
    
    def get_neighbor_list(self,node):
        if self.neighbor_dict.has_key(node):
            return self.neighbor_dict[node].keys()            
        else:
            return None
    
    def get_edge_weight(self,parent_name,child_name):
        return self._get_edge_element(parent_name,child_name,0)
    
    def get_edge_operation(self,parent_name,child_name):        
        return self._get_edge_element(parent_name,child_name,1)
                        
    def _get_edge_element(self,parent_name,child_name,index):        
        
        if self.has_node(parent_name) and self.has_node(child_name):
            parent = self.node_dict[parent_name] 
            child = self.node_dict[child_name]    
        
            if self.neighbor_dict.has_key(parent):
                neighbors = self.neighbor_dict[parent]
                if neighbors.has_key(child):
                    edge = neighbors[child]
                    return edge[index]                
        return None
    
    
    def has_node(self,node_name):
        return self.node_dict.has_key(node_name)
    
    #Creates a graphical representation of the nested tf_dictionary
    def to_image(self,filename="graph.png",path=[]):
        dot_graph = pydot.Dot(graph_type="digraph") #DIrected graph
        
        #Add all parents to the graph
        for parent in self.neighbor_dict.keys():
            if parent in path:
                dot_graph.add_node(pydot.Node(parent.name,style="filled", fillcolor="blue"))
            else:            
                dot_graph.add_node(pydot.Node(parent.name,style="filled", fillcolor="white"))
            
            for child in self.neighbor_dict[parent].keys():
                
                if child in path:    
                    dot_graph.add_node(pydot.Node(child.name,style="filled", fillcolor="blue"))
                    child_index = path.index(child)
                    
                    if child_index>0 and path[child_index-1] == parent:
                        dot_graph.add_edge(pydot.Edge(parent.name,child.name,color="blue"))        
                    else:
                        dot_graph.add_edge(pydot.Edge(parent.name,child.name))        
                    
                    
                else:
                    dot_graph.add_node(pydot.Node(child.name,style="filled", fillcolor="white"))
                    dot_graph.add_edge(pydot.Edge(parent.name,child.name))        
        
        dot_graph.write_png(filename)    
    

class GraphNode():
    def __init__(self,name):
        self.name = name    
    

class GraphSearchNode():
    def __init__(self,node,parent,cost=0):    
        self.node = node    
        self.parent=parent
        self.cost = cost

class GraphSearcher():
    def __init__(self):
        print "WARNING: This class shouldn't be directly instantiated."
    
    #Criterion to determine if a search node is the same as a goal
    #graph node.
    def is_same_node(self,search_node1,search_node2):
        if search_node1.node.name == search_node2.node.name:
            return True
        else:
            return False
    
    #Generic tree search algorithm
    def search(self,start,goal,graph):
        print "WARNING: Purely virtual function call."
        
    
    #Returns a path from start to goal
    def to_path(self,search_goal):
        search_current = search_goal;    path=[search_goal.node]
        
        while not (search_current.parent == None):            
            search_current = search_current.parent
            path.append(search_current.node)            
        
        path.reverse()                
        return path
    
    def get_op_sequence(self,node_path,graph):        
        return self._get_edge_sequence(node_path,graph.get_edge_operation)                

    def get_weight_sequence(self,node_path,graph):        
        return self._get_edge_sequence(node_path,graph.get_edge_weight)    
        
    def _get_edge_sequence(self,node_path,_graph_edge_fcn):        
        seq = []        
        for i in range(len(node_path)-1):
            el = _graph_edge_fcn(node_path[i].name,node_path[i+1].name)
            
            if el!=None:                
                seq.append(el)
            else:
                return None                
        return seq
    
    def print_path(self,path):
        for i in range(len(path)-1):
            sys.stdout.write(path[i].name+" -> ")        
        print path[len(path)-1].name
                

class GraphTreeSearcher(GraphSearcher):
    def __init__(self):    
        print "WARNING: This class shouldn't be directly instantiated."
    
    #Default implementation: this method should ideally 
    #be overriden by derived classes
    def update_queue(self,queue,new_elements):        
        print "WARNING: Purely virtual function call."
    
                
    #Generic tree search algorithm
    def search(self,start_name,goal_name,graph):
        
        start = graph.get_node(start_name)
        goal = graph.get_node(goal_name)
        
        if (start == None):
            print "Start node "+start_name+" isn't part of the graph."
            return []
        elif (goal == None):
            print "Goal node "+goal_name+" isn't part of the graph."
            return []
        
        queue = [GraphSearchNode(start,None)]    
        search_goal    = GraphSearchNode(goal,None)
        visited = []
        
        while not len(queue) == 0:
            
            current = queue.pop()
            
            if self.is_same_node(current,search_goal):                
                return self.to_path(current)
            else:        
                #Converts new nodes into search elements        
                new_elements = [GraphSearchNode(n,current,current.cost+graph.get_edge_weight(current.node.name,n.name)) for n in graph.get_neighbor_list(current.node) if not n in visited]
                
                queue = self.update_queue(queue,new_elements)                            
                visited.append(current.node)
        
        #No path found
        return []


class BFSSearcher(GraphTreeSearcher):
    def __init__(self):
        pass    
        
    def update_queue(self,queue,new_elements):        
        return (new_elements+queue)
        
        
class DFSSearcher(GraphTreeSearcher):
    def __init__(self):
        pass
            
    def update_queue(self,queue,new_elements):        
        return (queue+new_elements)

class UCSSearcher(GraphTreeSearcher):
    def __init__(self):
        pass
            
    def update_queue(self,queue,new_elements):                
        return sorted(queue+new_elements,reverse=True,key=lambda node: node.cost)

        
        
        
        
        
    
    
    
    
    






