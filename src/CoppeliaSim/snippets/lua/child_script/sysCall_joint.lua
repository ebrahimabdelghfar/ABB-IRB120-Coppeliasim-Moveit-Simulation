function sysCall_joint(inData)
    -- inData.mode : sim.jointmode_kinematic or sim.jointmode_dynamic
    --
    -- inData.handle : the handle of the joint to control
    -- inData.revolute : whether the joint is revolute or prismatic
    -- inData.cyclic : whether the joint is cyclic or not
    -- inData.lowLimit : the lower limit of the joint (if the joint is not cyclic)
    -- inData.highLimit : the higher limit of the joint (if the joint is not cyclic)
    -- inData.dt : the step size used for the calculations
    -- inData.pos : the current position
    -- inData.vel : the current velocity
    -- inData.targetPos : the desired position (if joint is dynamic, or when sim.setJointTargetPosition was called)
    -- inData.targetVel : the desired velocity (if joint is dynamic, or when sim.setJointTargetVelocity was called)
    -- inData.initVel : the desired initial velocity (if joint is kinematic and when sim.setJointTargetVelocity
    --                  was called with a 4th argument)
    -- inData.error : targetPos-currentPos (with revolute cyclic joints, the shortest cyclic distance)
    -- inData.maxVel : a maximum velocity, taken from sim.setJointTargetPosition or 
    --                 sim.setJointTargetVelocity's 3rd argument)
    -- inData.maxAccel : a maximum acceleration, taken from sim.setJointTargetPosition or
    --                   sim.setJointTargetVelocity's 3rd argument)
    -- inData.maxJerk : a maximum jerk, taken from sim.setJointTargetPosition or
    --                  sim.setJointTargetVelocity's 3rd argument)
    -- inData.first : whether this is the first call from the physics engine, since the joint
    --                was initialized (or re-initialized) in it.
    -- inData.passCnt : the current dynamics calculation pass. 1-10 by default
    -- inData.rk4pass : if Runge-Kutta 4 solver is selected, will loop from 1 to 4 for each inData.passCnt
    -- inData.totalPasses : the number of dynamics calculation passes for each "regular" simulation pass.
    -- inData.effort : the last force or torque that acted on this joint along/around its axis. With Bullet,
    --                 torques from joint limits are not taken into account
    -- inData.force : the joint force/torque, as set via sim.setJointTargetForce

    if inData.mode==sim.jointmode_dynamic then
        -- a simple position controller
        local ctrl=inData.error*20
        local maxVelocity=ctrl
        if (maxVelocity>inData.maxVel) then
            maxVelocity=inData.maxVel
        end
        if (maxVelocity<-inData.maxVel) then
            maxVelocity=-inData.maxVel
        end
        local forceOrTorqueToApply=inData.maxForce
        local outData={vel=maxVelocity,force=forceOrTorqueToApply}
        return outData
    end
    -- Expected return data:
    -- For kinematic joints:
    -- outData={pos=pos, vel=vel, immobile=false}
    -- 
    -- For dynamic joints:
    -- outData={force=f, vel=vel}
end
