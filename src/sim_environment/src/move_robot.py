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

# this function is called to set the joint angle of the robot in the simulator received from moveit
def joint_callback(data:JointState):
  '''
  def joint_callback(data:JointState):
    argument : 
      data : JointState message recieved from moveit
    function :
      set the joint angle of the robot in the simulator
    return:
      None
'''
  global ID,joint
  for i in range(6):
    # get the joint handle of the each joint and give the angle to the joint
    _,joint_handle=sim.simxGetObjectHandle(ID,joint[i],sim.simx_opmode_oneshot_wait)
    # now move the joint to the angle received from moveit
    sim.simxSetJointTargetPosition(ID,joint_handle,data.position[i],sim.simx_opmode_oneshot)

#intiate the node 
rospy.init_node("move_robot")
#define the subscriber
rospy.Subscriber("/move_group/fake_controller_joint_states",JointState,joint_callback)
#defining the port number and getting the ID number
ID=connect_port(19999)
#keep the node running
rospy.spin()  