function SetCoppeliaMotorPositions(sim, clientID, joint_names, joint_angles)

% Ensure that the joint names are given as a cell.
if ischar(joint_names)                  % If the joint names variable is a character string...
   
    % Convert the character array to a cell.
    joint_names = {joint_names};
    
end

% Ensure that the number of joint angles matches the number of joint names.
assert(length(joint_names) == length(joint_angles), 'Number of joint names must match the number of desired joint angles.')

% Determine the number of joints.
num_joints = length(joint_names);

% Set the target position of each joint.
for k = 1:num_joints                    % Iterate through each joint...

    % Set the target position of this joint.
    SetCoppeliaMotorPosition(sim, clientID, joint_names{k}, joint_angles(k))

end


end

