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

def joint_callback(data:JointState):
    global ID,joint
    for i in range(6):
      _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_oneshot_wait)
      sim.simxSetJointTargetPosition(ID,joint_handle,data.position[i],sim.simx_opmode_oneshot)


rospy.init_node("move_robot")

rospy.Subscriber("/move_group/fake_controller_joint_states",JointState,joint_callback)

ID=connect_port(19999)

rospy.spin()




  