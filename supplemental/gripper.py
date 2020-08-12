#!/usr/bin/env python

# Python includes
import numpy

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Vector3
from tf import transformations

from rviz_tools_py import rviz_tools

# Initialize the ROS Node
rospy.init_node('marker_gripper', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/tool0', 'gripper_marker')

while not rospy.is_shutdown():
  T = transformations.translation_matrix((0,0,0))
  scale = Vector3(1,1,1)
  mesh_file1 = "package://rovi_industrial/mesh/Gripper.stl"
  markers.publishMesh(T, mesh_file1, 'grey', scale, 0.5)
  rospy.Rate(5).sleep() #1 Hz
