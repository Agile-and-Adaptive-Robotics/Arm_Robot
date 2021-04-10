function SetCoppeliaMotorPosition(sim, clientID, joint_name, joint_angle)

% Retrieve the joint handle associated with this joint.
[~, joint_handle] = sim.simxGetObjectHandle(clientID, joint_name, sim.simx_opmode_blocking);

% Set the target position of this joint.
sim.simxSetJointTargetPosition(clientID, joint_handle, joint_angle, sim.simx_opmode_oneshot);

end

