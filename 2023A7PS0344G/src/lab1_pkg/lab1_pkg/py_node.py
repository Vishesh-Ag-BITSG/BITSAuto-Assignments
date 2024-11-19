#!/usr/bin/env python3
#interpreter line - tells the interpreter to use python3

import rclpy 
# Python lib for ROS2 - 
# needs to be imported everytime we create a node with python

from rclpy.node import Node

class MyNode(Node):
    # Nodes are created using object oriented programming
    # 'MyNode' class inherits from the 'Node' class of rclpy

    def __init__(self): 
        # Constructor

        super().__init__("py_node_name")
        # 'Node' is the parent class
        # Constructor of 'Node' class is called
        # 'py_node_name' is the name of the node
        # 'py_node.py' is the name of program/file
        # Now, we have initialised the node

        #self.get_logger().info("Hello from Py Node")
            # get_logger() comes from the 'Node' class
            # Inheritance concepts applied here where self. can access these funcionalities

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello Py")


def main(args=None):
# 'args' allows main function to take some arguements
# By default, args = None
# Useful when we want to install the node with ROS2 functionalities

    rclpy.init(args=args) 
    # Initialize ros2 communications: Prepares environment for nodes to interact
    # Sets up necessary communication infrastructure, so that ROS2 nodes can send and receive data
    # 'args' parameter of init function is equal to args that we get from main function

    # Nodes are constructed between rclpy.init() and rclpy.shutdown()
    # ROS2 communication is only possible in the space between these 2
    # Node is not the program itself, the node will be created inside the program
    # It is possible to create multiple nodes in the same program

    node = MyNode() 
    # We have now created an instance of the MyNode class (which inherits from Node class)
    # There are no arguements in the constructor, so nothing in paranthesis here


    rclpy.spin(node)
    # When you make a node 'spin' the node is going to be kept alive
    # While it is alive, al; the callbacks will be able to run
    # When you kill the node with ctrl+c, .spin() returns and then proceeds to the next line

    rclpy.shutdown() 
    # Stops ros2 communications
    # Cleans up the resources aalocated by rclpy.init()

if __name__ == '__main__':
    main()
# This is useful when we execute the file from the terminal by running python script

# We want to run the node with ROS2 functionalities from terminal
# For this, we will need to 'install' the node
# Without this, all we can do is run this file as an executable which in turn will run the node

# When we 'install' the node we create an executable which is called/executed in the terminal
# So, 3 things to think about:
    # a) File name, here it is py_node.py
    # b) Node name, here it is py_node_name
    # c) Executable name, not set here since this is a hybrid package