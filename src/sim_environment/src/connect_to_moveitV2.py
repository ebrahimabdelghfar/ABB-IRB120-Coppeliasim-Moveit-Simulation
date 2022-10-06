#!/bin/env python3
import rospy
import sim
from sensor_msgs.msg import JointState
import tf2_ros
import geometry_msgs.msg

joint=["joint1","joint2","joint3","joint4","joint5","joint6"]

def connect_port(port):
  sim.simxFinish(-1)
  clientID=sim.simxStart("127.0.0.1",port,True,True,2000,5) #get the client ID
  if clientID==0:
    print("connected")
  else:
    print ("disconnected")
  return clientID

ID=connect_port(19997)

def joint_callback(data:JointState):
    global ID,joint
    for i in range(6):
      _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_oneshot_wait)
      sim.simxSetJointTargetPosition(ID,joint_handle,data.position[i],sim.simx_opmode_oneshot)

def get_joint_position():
  pos=[]
  for i in range(6):
    _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_continuous)
    _,p=sim.simxGetJointPosition(ID,joint_handle,sim.simx_opmode_continuous)
    pos.append(p)
  return pos

def getTransformStamped(obj_handle,name,relTo,relToName):
  t = geometry_msgs.msg.TransformStamped()
  _,p=sim.simxGetObjectPosition(ID,obj_handle,relTo,sim.simx_opmode_streaming)
  _,o=sim.simxGetObjectQuaternion(ID,obj_handle,relTo,sim.simx_opmode_streaming)
  t.header.frame_id=relToName
  t.header.stamp=rospy.Time.now()
  t.child_frame_id = name
  t.transform.translation.x = p[0]
  t.transform.translation.y = p[1]
  t.transform.translation.z = p[2]
  t.transform.rotation.x = o[0]
  t.transform.rotation.y = o[1]
  t.transform.rotation.z = o[2]
  t.transform.rotation.w = o[3]
  return t

rospy.init_node("connector")

rospy.Subscriber("/move_group/fake_controller_joint_states",JointState,joint_callback)

joint_pub=rospy.Publisher("/joint_states",JointState,queue_size=1)

br = tf2_ros.TransformBroadcaster()


while not rospy.is_shutdown():
  pos=get_joint_position()
  let_send=JointState()
  let_send.header.stamp=rospy.Time.now()
  let_send.name=joint
  let_send.position=pos
  joint_pub.publish(let_send)
  # for i in range(6):
  #   _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_oneshot_wait )
  #   br.sendTransfor(getTransformStamped(joint_handle,joint[i],-1,'world'))




  