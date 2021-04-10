function WriteCoppeliaMotorPositions(sim, clientID, joint_names, joint_trajectories)

% Retrieve the number of points in our trajectory.
num_points = size(joint_trajectories, 2);

% Pass the joint angles associated with each trajectory point to CoppeliaSim
for k = 1:num_points                    % Iterate through each trajectory point...
    
    % Send the joint angles associated with this point to CoppeliaSim.
    SetCoppeliaMotorPositions(sim, clientID, joint_names, joint_trajectories(:, k))

end

end

