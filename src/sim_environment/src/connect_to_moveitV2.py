#!/bin/env python3
import rospy
import sim
from sensor_msgs.msg import JointState

joint=["joint1","joint2","joint3","joint4","joint5","joint6"]

def connect_port(port):
  sim.simxFinish(-1)
  clientID=sim.simxStart("127.0.0.1",port,True,True,2000,5) #get the client ID
  if clientID==0:
    print("connected")
  else:
    print ("disconnected")
  return clientID

ID=connect_port(19999)

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

rospy.init_node("connector")

rospy.Subscriber("/move_group/fake_controller_joint_states",JointState,joint_callback)

joint_pub=rospy.Publisher("/joint_states",JointState,queue_size=1)

while not rospy.is_shutdown():
  pos=get_joint_position()
  let_send=JointState()
  let_send.header.stamp=rospy.Time.now()
  let_send.name=joint
  let_send.position=pos
  joint_pub.publish(let_send)





  