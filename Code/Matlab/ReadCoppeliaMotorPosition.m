function joint_angle = ReadCoppeliaMotorPosition(sim, clientID, joint_name)

% Retrieve the joint handel associated with this joint.
[~, joint_handle] = sim.simxGetObjectHandle(clientID, joint_name, sim.simx_opmode_blocking);

% Retrieve the joint angle associated with this joint.
[~, joint_angle] = sim.simxGetJointPosition(clientID, joint_handle, sim.simx_opmode_blocking);

end

