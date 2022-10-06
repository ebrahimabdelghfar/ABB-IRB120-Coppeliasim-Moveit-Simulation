function subscriber_callback(msg)
	for i=1,#msg.position,1 do
		sim.setJointPosition(jointHandles[i],msg.position[i])
	end
end

function getTransformStamped(objHandle,name,relTo,relToName)

    t=sim.getSystemTime()

    p=sim.getObjectPosition(objHandle,relTo)

    o=sim.getObjectQuaternion(objHandle,relTo)

return {

        header={

            stamp=t,

            frame_id=relToName

        },

        child_frame_id=name,

        transform={

            translation={x=p[1],y=p[2],z=p[3]},

            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}

        }

    }

end

function getJointStates()

for i=1,6,1 do

        p = sim.getJointPosition(jointHandles[i])

        jointPosition[i] = p

end

end

function sysCall_init()

    jointHandles={-1,-1,-1,-1,-1,-1}

    jointNames = {'joint1','joint2','joint3','joint4','joint5','joint6'}

    jointPosition = {-1,-1,-1,-1,-1,-1}

for i=1,6,1 do

        jointHandles[i]=sim.getObjectHandle('joint'..i)

end

    rosInterfacePresent=simROS

if rosInterfacePresent then

        publisher=simROS.advertise('/joint_states','sensor_msgs/JointState')

        subscriber=simROS.subscribe('/move_group/fake_controller_joint_states','sensor_msgs/JointState','subscriber_callback')

end

end

function sysCall_actuation()

if rosInterfacePresent then

        getJointStates()

        simROS.publish(publisher,{header = {stamp =  sim.getSystemTime()} ,name = jointNames, position=jointPosition})

for i=1, 6, 1 do 

            simROS.sendTransform(getTransformStamped(jointHandles[i],sim.getObjectName(jointHandles[i]),-1,'world'))

end

end

end

function sysCall_sensing()

end

function sysCall_cleanup()

if rosInterfacePresent then

        simROS.shutdownPublisher(publisher)

        simROS.shutdownSubscriber(subscriber)

end

end
