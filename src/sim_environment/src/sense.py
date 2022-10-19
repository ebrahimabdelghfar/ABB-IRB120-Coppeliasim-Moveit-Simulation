#!/bin/env python3
import rospy
import sim
from sensor_msgs.msg import JointState
# defining the name of robot joints to be controlled by ROS
joint=["joint1","joint2","joint3","joint4","joint5","joint6"]

# this function is called at the start to get the ID number for simulator
'''
def connect_port(port):
  argument :
    port : port number of the simulator
  return :
    clientID : ID number of the simulator
'''
def connect_port(port):
  sim.simxFinish(-1)
  clientID=sim.simxStart("127.0.0.1",port,True,True,2000,5) #get the client ID
  if clientID==0:
    print("connected")
  else:
    print ("disconnected")
  return clientID

# defining the port number and getting the ID number
ID=connect_port(19998)

# this function is used to get the joint angle of the robot in the simulator 
'''
def get_joint_position():
  argument :
    None
  function :
    get the joint angle of the robot in the simulator
  return:
    pos : list of joint angles to be published to moveit
'''
def get_joint_position():
  pos=[]
  for i in range(6):
    _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_continuous)
    _,p=sim.simxGetJointPosition(ID,joint_handle,sim.simx_opmode_continuous)
    pos.append(p)
  return pos

# initate the node the moveit 
rospy.init_node("sensing")

# initate the publisher
'''
argeument : 
  topic name : /joint_states
  message type : JointState
  queue size : 0 "to ensure that theres no old message in the queue"
'''
joint_pub=rospy.Publisher("/joint_states",JointState,queue_size=0)

# now publish the joint angle to moveit
while not rospy.is_shutdown():
  pos=get_joint_position()
  let_send=JointState() # create a JointState message
  let_send.header.stamp=rospy.Time.now() # set the time stamp for the current time
  let_send.name=joint # set the name of the joints
  let_send.position=pos # set the position of the joints
  joint_pub.publish(let_send) # publish the message






  