#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib
from scipy.spatial.transform import Rotation as R

def lookup(a,b):
  try:
    aTb=tfBuffer.lookup_transform(a,b,rospy.Time(0))
  except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
    return None
  return aTb

def cb_update():
  stj=lookup("camera/master0","camera/master0/journal")
  stj.header.frame_id="camera/capture0/solve0"
  stj.child_frame_id="camera/capture0/solve0/journal"
  cts=lookup("camera/capture0","camera/capture0/solve0")
  cTs=tflib.toRT(cts.transform)
  sTj=tflib.toRT(stj.transform)
  jTs=sTj.I
  jTc=jTs.dot(cTs.I)
  bsz=np.ravel(jTs[:3,2])
  bcz=np.ravel(jTc[:3,2])
  bsz[0]=0
  bcz[0]=0
  bsz=bsz/np.linalg.norm(bsz)
  bcz=bcz/np.linalg.norm(bcz)
  rot=np.cross(bsz,bcz)
  lrot=np.linalg.norm(rot)
  arot=np.arcsin(lrot)
  if lrot>0:
    rot=rot/lrot*arot
  r=R.from_rotvec(rot)
  RT=np.eye(4)
  RT[:3,:3]=r.as_dcm()
  jTr=RT.dot(jTs)
  jtr=TransformStamped()
  jtr.header.frame_id="camera/capture0/solve0/journal"
  jtr.child_frame_id="camera/capture0/solve0/revolve"
  jtr.header.stamp=rospy.Time.now()
  jtr.transform=tflib.fromRT(jTr)
  broadcaster.sendTransform([stj,jtr])

########################################################
rospy.init_node('revolver',anonymous=True)
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

sys.stdout.write("sys revolve.py started")
sys.stdout.flush()

while not rospy.is_shutdown():
  line=sys.stdin.readline()
  cb_update()
  sys.stdout.write("revolve.py update")
  sys.stdout.flush()
