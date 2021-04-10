%% ME 557: Project, Control Robot.

%This script is the central script that continuously runs, checking for user input via the xbox controller and sending the appropriate signals to the arduino.

%SETWRITEPLANE: DO NOT ALLOW USER TO SUPPLY DUPLICATE POINTS.
%SETWRITEPLANE: ALLOW USER TO BREAK OUT OF PLANE DEFINITION PROCEDURE.
%CONTROLLERMOVEMENT: ALLOW D-PAD MOTIONS TO CREATE MOTIONS IN THE PLANE.
%CONTROLLERMOVEMENT: ALLOW BUMPERS TO MOVE PERPENDICULAR TO THE PLANE.
%CONTROLLERMOVEMENT: ALLOW GRIPPER TO OPEN AND CLOSE WITH B-BUTTON.

%Increase the scaling factor to write as large of letters as possible that are within the workspace.
%If clarity is still not satisfactory, increase the middle length link and continue to increase the scaling factor until clarity is satisfactory.

%Clear Everything
clear, close('all'), clc

%% Open Serial Port.

% Open the Blue Zero Serial Port connection.
[sim, clientID] = OpenBlueZeroConnection();


%% Define the Geometry of the Robot.

%Define the link lengths of the robot.
% rs = [2.047244, 1.417323, 5.905512, 4.074803, 1.633858, 2.657480, 3.972441];
% rs = [2.047244, 1.417323, 5.905512, 4.074803, 1.633858, 2.657480, 4.322441];
% rs = [2.047244, 1.417323, 5.905512, 4.074803, 1.633858, 2.657480, 4.25];
rs = [2.047244, 1.417323, 5.905512, 4.074803, 1.633858, 2.657480, 4.30];

%Define the home orientation of each joint.  To compute the appropriate joint angles, we only need the home orientation of the end effector.  However, in order to plot the movement of all of the joints, we need all home orientations.
M1 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M2 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2); 0 0 0 1];
M3 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3); 0 0 0 1];
M4 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4); 0 0 0 1];
M5 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5); 0 0 0 1];
M6 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6); 0 0 0 1];
M7 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6) + rs(7); 0 0 0 1];

% %Store the joint orientations in a multidimensional matrix.  Each layer corresponds to one of the joint home positions.
M = cat(3, M1, M2, M3, M4, M5, M6, M7);

% Define the robot rotation axes.
% w1 = [0; 0; 1];
% w2 = [1; 0; 0];
% w3 = [1; 0; 0];
% w4 = [0; 0; 1];
% w5 = [0; -1; 0];
% w6 = [1; 0; 0];

w1 = [0; 0; 1];
w2 = [1; 0; 0];
w3 = [-1; 0; 0];
w4 = [0; 0; 1];
w5 = [0; 1; 0];
w6 = [-1; 0; 0];

% Define the displacement vectors to each joint.
r1 = [0; 0; rs(1)];
r2 = [0; 0; rs(1) + rs(2)];
r3 = [0; 0; rs(1) + rs(2) + rs(3)];
r4 = [0; 0; rs(1) + rs(2) + rs(3) + rs(4)];
r5 = [0; 0; rs(1) + rs(2) + rs(3) + rs(4) + rs(5)];
r6 = [0; 0; rs(1) + rs(2) + rs(3) + rs(4) + rs(5) + rs(6)];

% Compute the velocity components associated with each joint.
v1 = cross(r1, w1);
v2 = cross(r2, w2);
v3 = cross(r3, w3);
v4 = cross(r4, w4);
v5 = cross(r5, w5);
v6 = cross(r6, w6);

%Define the screw axes.
S1 = [w1; v1];
S2 = [w2; v2];
S3 = [w3; v3];
S4 = [w4; v4];
S5 = [w5; v5];
S6 = [w6; v6];

%Store the screw axes in a matrix.  Each column of S is a screw axis for a different joint.
S = [S1 S2 S3 S4 S5 S6];


%% Initialize Simulation Parameters.

% Define the CoppeliaSim joint names.
joint_names = {'MX64_Joint1', 'MX64_Joint2', 'AX12_Joint1', 'AX12_Joint2', 'AX12_Joint3', 'AX12_Joint4'};

% Retrieve the joint handle associated with this joint.
[~, joint_handles] = sim.simxGetObjectHandle(clientID, joint_name, sim.simx_opmode_blocking);

%Set the default letter.
% Ltr = 'ABCDE';
Ltr = 'PSU';

%Define the array of possible letters.
Ltrs = {'A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X' 'Y' 'Z'};

%Define the rotational and translational tolerances to use for the Newton-Raphson approximation of the inverse kinematics.
% [eomg, ev] = deal( 1, 1e-2 );
[eomg, ev] = deal( 1e-3, 1e-3 );

%Define the maximum displacement in a single step.
% [dtol_ltr, dtol_travel] = deal( 1, 0.75 );         %Works okay in regular mode.
% [dtol_ltr, dtol_travel] = deal( 1/8, 1/4 );         %Works okay in regular mode.
[dtol_ltr, dtol_travel] = deal( 1/16, 1/8 );         %Works okay in regular mode.

%Set the making plots option to false.  Some functions make a lot of different plots for reference and debugging, however it isn't always useful to have many plots appearing.
bMakePlots = false;

%Set whether to animate the letter trajectory.
bAnimateLetterTrajectory = true;

%Set whether to create the motor plots.
bPlotMotorGraphs = false;

%Set the movement speed.
move_speed = 0.1;

%Set the assumed starting angle.
% [thetac, theta_home] = deal( [0 0 0 0 0 0]' );
[thetac, theta_home] = deal( (pi/180)*[0 45 90 0 0 45]' );

%Offset the starting current motor angles.  This prevents the system from moving very slowly the first time the system is sent to the home position.
thetac = thetac + pi/6;

%Define the Default Plane Points.
% Pts_Plane =    [-3.0   0.0    3.0;
%                 16.0   16.0   16.0;
%                 3.0    6.0    3.0];

% Pts_Plane =    [-3.0   0.0    3.0;
%                 16.0   16.0   16.0;
%                 5.5    8.5    5.5];

Pts_Plane =    [-3.0   0.0    3.0;
                16.0   16.0   16.0;
                5.75    8.75    5.75];
            
%Compute the transformation matrix associated with the defined plane.  i.e., the transformation matrix that maps from the template letter space in the xy-plane at the origin to the location of the newly defined plane in space.
T_Plane = GetPlaneTransformationMatrix( Pts_Plane, [0 1 0]' );


%% Return to the Home Position.

% Send the robot to its home position.
WriteCoppeliaMotorPositions(sim, clientID, joint_names, theta_home)

%Reset the default motor angles.
thetac = theta_home;

%Retrieve the current orientation of the end effector.
Tc = FKinSpace(M(:, :, end), S, thetac);


%% Generate the Trajectory.

%State that letter trajectories will now be generated.
fprintf('\nGENERATING TRAJECTORY. Please Wait...\n\n')

%State which letter is being processed.
fprintf(['Generating ', Ltr, ' Trajectory...\n'])

%Generate the letter trajectories.
[ thetas, mpos ] = PlanLetterTrajectoryWithTravel( S, M, Tc, T_Plane, Pts_Plane, Ltr, thetac, eomg, ev, dtol_ltr, dtol_travel, bMakePlots );

%Print that we are finished generating the letter trajectories.
fprintf('\nDone: Letter Trajectories Confirmed.\n\n')


%% Animate the Trajectory.

%Animate the trajectory, if requsted.
if bAnimateLetterTrajectory                                 %If we want to animate the letter writting trajectory...

    %State that we are going to animate the trajectory.
    fprintf('\nTRAJECTORY ANIMATION\n')
    fprintf('Animating trajectory...\n')

    %Animate the given trajectory.
    AnimateTrajectoryFunc( S, M, T_Plane, Pts_Plane, rs, thetas )

    %State that we are done animating the trajectory.
    fprintf('\nDone: Animating trajectory.\n')

end

%% Plot the Joint Angles.

fig = figure('Color', 'w'); hold on, grid on, xlabel('Point Number [#]'), ylabel('Joint Angle [deg]'), title('Joint Angle vs Point Number')
plot(1:size(thetas, 2), (180/pi)*thetas, '-', 'Linewidth', 3)
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'})


%% Write Motor Positions to CoppeliaSim.

% Ensure that the target angles are mapped to the correct range [-180, 180) for our motors (rather than [0, 360)).
thetas = thetas - 2*pi*(thetas > pi);

% Ensure that the AX12 motors are limited to be in-bounds.
for k = 1:4

    thetas(k + 2, thetas(k + 2, :) < -165*(pi/180)) = -165*(pi/180);
    thetas(k + 2, thetas(k + 2, :) > 165*(pi/180)) = 165*(pi/180);

end

% Send the joint trajectories to CoppeliaSim.
WriteCoppeliaMotorPositions(sim, clientID, joint_names, thetas)

%Update the current motor angles variable.
thetac = thetas(:, end);


%% Return to the Home Position.

% Send the robot to its home position.
WriteCoppeliaMotorPositions(sim, clientID, joint_names, theta_home)

%Reset the default motor angles.
thetac = theta_home;


%% Close Serial Port.

% Close the Blue Zero connection.
CloseBlueZeroConnection(sim, clientID)

