#!/usr/bin/env python3
# 2021/03/17 hato #!/usr/bin/env python -> !/usr/bin/env python3

# Python includes
import numpy

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Vector3
from tf import transformations

from rviz_tools_py import rviz_tools

# Initialize the ROS Node
rospy.init_node('marker_tp001', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    # 2021/03/16 hato ------------------------------ start ------------------------------
    # print "Shutting down node"
    print("Shutting down node")
    # 2021/03/16 hato ------------------------------  end  ------------------------------
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/world', 'tp001_marker')

while not rospy.is_shutdown():
  T = transformations.translation_matrix((500,0,340))
  scale = Vector3(1,1,1)
  mesh_file1 = "package://rovi_industrial/mesh/TP001.stl"
  markers.publishMesh(T, mesh_file1, 'white', scale, 0.5)
  rospy.Rate(5).sleep() #1 Hz
