import sys
import copy
import rospy
from moveit_commander import *
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf.transformations
import math

rospy.init_node("ABB_IRB120")

class ABB_IRB120:
    def __init__(self,group_name="joint_group"):
        '''
        --------------------
        This class is used to control the ABB IRB120 robot using moveit
        --------------------
        constructor arguments:
            node_name: name of the node
            group_name: name of the group of joints to be controlled by moveit
        '''
        # initialize moveit_commander and rospy node
        roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object.
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = RobotCommander()

        # Instantiate a PlanningSceneInterface object.
        # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = PlanningSceneInterface()

        # define the group of joints to be controlled by moveit
        self.move_group = MoveGroupCommander(group_name)

        self.move_group.set_planning_time(0.05)
        self.move_group.allow_replanning(True)
        pass
    def go_by_joint_angle(self, joint_goal_list,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired joint state
        --------------------
        arguments:
            joint_goal: list of joint angles in radians
            velocity: velocity of the robot
            acceleration: acceleration of the robot
        '''
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal joint state
        self.move_group.go(joint_goal_list,wait=True)
        # stop any residual movement
        self.move_group.stop()
        pass
    def go_to_pose_goal_cartesian(self, pose_goal,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            pose_goal: geometry_msgs.msg.Pose
            velocity: velocity of the robot
            acceleration: acceleration of the robot
        '''
        # set the velocity of the robot
        self.move_group.set_max_velocity_scaling_factor(velocity)
        # set the acceleration of the robot
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # set the goal pose
        self.move_group.set_pose_target(pose_goal)
        # plan the motion
        plan = self.move_group.go(wait=True)
        # clear the targets
        self.move_group.clear_pose_targets()
        pass
    def go_to_pose_goal_cartesian_waypoints(self, waypoints,velocity=0.1,acceleration=0.1):
        '''
        --------------------
        This function is used to move the robot to the desired pose by cartesian path
        --------------------
        arguments:
            waypoints: nx6 list of waypoints
                n: number of waypoints
                6: x,y,z,roll,pitch,yaw
            
            velocity: velocity of the robot
        
            acceleration: acceleration of the robot
        '''
        list_of_poses = []
        geo_pose=geometry_msgs.msg.Pose() #create a geometry_msgs.msg.Pose() object

        #its better to use the current pose of the robot as the starting point or execute will fail 
        current=self.move_group.get_current_pose().pose #get the current pose of the robot
        list_of_poses.append(copy.deepcopy(current)) # append the current pose to the list of poses as start
        for ways in waypoints:
            #set the position of the pose
            geo_pose.position.x = ways[0]
            geo_pose.position.y = ways[1]
            geo_pose.position.z = ways[2]
            #set the orientation of the pose
            quantrion = tf.transformations.quaternion_from_euler(ways[3],ways[4],ways[5])
            geo_pose.orientation.x = quantrion[0]
            geo_pose.orientation.y = quantrion[1]
            geo_pose.orientation.z = quantrion[2]
            geo_pose.orientation.w = quantrion[3]
            #append the pose to the list
            list_of_poses.append(copy.deepcopy(geo_pose))

        # set the goal pose
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                    list_of_poses,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        # plan the motion

        # generate a new plan with the new velocity and acceleration by retiming the trajectory
        new_plan=self.move_group.retime_trajectory(self.robot.get_current_state(),plan,velocity_scaling_factor=velocity,acceleration_scaling_factor=acceleration)
        
        # execute the plan
        self.move_group.execute(new_plan,wait=True)

        pass
    def generate_spiral_waypoints(self,starting_pose,angle,step):
        '''
        --------------------
        This function is used to generate a list of waypoints for a spiral path
        --------------------
        arguments:
            starting_pose: geometry_msgs.msg.Pose
            angle: angle of the spiral in radians
            step: step size of the spiral
        --------------------
        return:
            list_of_poses: nx6 list of waypoints
                n: number of waypoints
                6: x,y,z,roll,pitch,yaw
        '''
        list_poses = []
        # generate the waypoints
        for i in range(angle):
            list_poses.append([starting_pose.position.x+step*i*math.cos(i/5),starting_pose.position.y+step*i*math.sin(i/5),current_pose.position.z,math.pi,0,0])
        return list_poses

    def get_joint_state(self):
        '''
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current joint state of the robot
        --------------------
        return:
            joint_state: list of joint angles in radians
        --------------------
        '''
        # get the current joint state
        joint_state = self.move_group.get_current_joint_values()
        return joint_state
    def get_pose(self):
        '''
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current pose of the robot
        --------------------
        return:
            pose: geometry_msgs.msg.Pose
        --------------------
        '''
        # get the current pose
        pose = self.move_group.get_current_pose().pose
        return pose
    def get_joint_velocity(self):
        '''
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current joint velocity of the robot
        --------------------
        return:
            joint_velocity: list of joint velocities in radians per second
        --------------------
        '''
        # get the current joint velocity
        joint_velocity = self.move_group.get_current_joint_velocity()
        return joint_velocity
    def get_end_effector_velocity(self):
        '''
        --------------------
        arguments:
            no arguments
        --------------------
        This function is used to get the current end effector velocity of the robot
        --------------------
        return:
            end_effector_velocity: geometry_msgs.msg.Twist
        --------------------
        '''
        # get the current end effector velocity
        end_effector_velocity = self.move_group.get_current_velocity()
        return end_effector_velocity

class frames_transformations:
    '''
     functioninality:
        This class is used in transforming and putting the frames using tf2_ros
    '''
    def __init__(self):
        '''
        --------------------
        This class is used to transform the frames
        --------------------
        '''
        # This object is used to store the frames
        self.tfBuffer = tf2_ros.Buffer()

        # instantiate a tf2_ros.TransformListener object
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # instantiate a tf2_ros.StaticTransformBroadcaster object
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # instantiate a tf2_ros.TransformBroadcaster object
        self.broadcaster = tf2_ros.TransformBroadcaster()

        pass

    def transform(self, parent_id, child_frame_id):
        '''
        --------------------
        This function is used to transform the frames
        --------------------
        arguments:
            parent_id: name of the parent frame
            child_frame_id: name of the child frame
        --------------------
        return:
            pose: geometry_msgs.msg.PoseStamped()
        '''
        # transform the frame

        transform_msg = geometry_msgs.msg.TransformStamped()
        pose=geometry_msgs.msg.Pose()

        transform_msg = self.tfBuffer.lookup_transform(parent_id,child_frame_id,rospy.Time.now())
        #transfer from TransformStamped() to PoseStamped()
        pose.position.x = transform_msg.transform.translation.x
        pose.position.y = transform_msg.transform.translation.y
        pose.position.z = transform_msg.transform.translation.z
        pose.orientation.x = transform_msg.transform.rotation.x
        pose.orientation.y = transform_msg.transform.rotation.y
        pose.orientation.z = transform_msg.transform.rotation.z
        pose.orientation.w = transform_msg.transform.rotation.w

        return pose

    def put_frame_static_frame(self,parent_frame_name="base_link",child_frame_name="tool0",frame_coordinate=[0,0,0,0,0,0]):
        '''
        --------------------
        This function is used to put the frame in the tf tree
        --------------------
        arguments:
            parent_frame_name: name of the parent frame
            child_frame_name: name of the child frame
            frame_coordinate: list of coordinates of the frame 1x6
                [x,y,z,rx,ry,rz]
                translation( in meter ) and rotation ( in radians )

            
        '''
        frames_msg=geometry_msgs.msg.TransformStamped()
        frames_msg.header.frame_id=parent_frame_name
        frames_msg.child_frame_id=child_frame_name
        frames_msg.transform.translation.x=frame_coordinate[0]
        frames_msg.transform.translation.y=frame_coordinate[1]
        frames_msg.transform.translation.z=frame_coordinate[2]
        quatrion=tf.transformations.quaternion_from_euler(frame_coordinate[3],frame_coordinate[4],frame_coordinate[5])
        frames_msg.transform.rotation.x=quatrion[0]
        frames_msg.transform.rotation.y=quatrion[1]
        frames_msg.transform.rotation.z=quatrion[2]
        frames_msg.transform.rotation.w=quatrion[3]
        # put the frame in the tf tree
        self.static_broadcaster.sendTransform(frames_msg)

# main_function for testing the class
if __name__ == "__main__":
    ABB_robot = ABB_IRB120()
    frames=frames_transformations()
    current_pose=geometry_msgs.msg.Pose()
    current_pose=ABB_robot.get_pose()
    list_of_poses=list()
    # produce list of spiral cordinate (x,y) for the robot to follow strat from current pose
    list_of_poses=ABB_robot.generate_spiral_waypoints(current_pose,360,0.001)
    # put the spiral list as frames
    for i in range(0,200):
        frames.put_frame_static_frame(parent_frame_name="base_link",child_frame_name="frame_"+str(i),frame_coordinate=list_of_poses[i])
        rospy.sleep(0.001)
    # move the robot follow waypoints
    ABB_robot.go_to_pose_goal_cartesian_waypoints(list_of_poses,,0.001,0.001)



    

    






 


    