function joint_angles = ReadCoppeliaMotorPositions(sim, clientID, joint_names)

% Ensure that the joint names are given as a cell.
if ischar(joint_names)                  % If the joint names variable is a character string...
   
    % Convert the character array to a cell.
    joint_names = {joint_names};
    
end

% Determine the number of joints.
num_joints = length(joint_names);

% Preallocate an array to store the joint angles.
joint_angles = zeros(num_joints, 1);

for k = 1:num_joints

    % Retrieve the motor position associated with this joint.
    joint_angles(k) = ReadCoppeliaMotorPosition(sim, clientID, joint_names{k});

end

end

