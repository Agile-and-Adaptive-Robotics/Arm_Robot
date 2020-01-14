%% ME 557: Project, Animate Trajectories.

%Given link lengths and motor angles, this script computes the link point
%trajectories and animates them.

%Clear Everything
clear, close('all'), clc

%% Define the Letter of Interest.

%Define the letter of interest.
Ltr = 'E';

%% Define the white board points.

%Define the distance to the white board.
d = 18;

%Define the width of the White Board.
r = 50;

%Parameterize a plane.
xs = linspace(-r, r, 100);
zs = linspace(-r, r, 100);

%Convert to matrices.
[Xs, Zs] = meshgrid(xs, zs);

%The Y values of the whiteboard should be constant, since we want the
%whiteboard to be parallel to the zx-plane.
Ys = d*ones(length(xs), length(zs));

%Create a 3D matrix to store the white board points.
[WBs(:, :, 1), WBs(:, :, 2), WBs(:, :, 3)] = deal( Xs, Ys, Zs);


%% Compute the Link Point Trajectories.

%Define the path to the desired angles file.
RawTrajFolder = 'C:\Users\USER\Documents\Coursework\MSME\Year1\Winter2018\ME557_IntroToRobotics\Project\Trajectories';
fname_RawMotorAngles = strcat('MotorAnglesTrajectory_', Ltr, '.txt');
fpath_RawMotorAngles = strcat(RawTrajFolder, '\', fname_RawMotorAngles);

%Read in the desired motor angles.
thetas = dlmread(fpath_RawMotorAngles);

%Define the link lengths of the robot.
rs = [7 7 3 7 3 7];

%Define the orientation of each joint.
M2 = [1 0 0 0; 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M3 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 rs(1); 0 0 0 1];
M4 = [1 0 0 rs(2); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M5 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3)); 0 0 0 1];
M6 = [1 0 0 (rs(2) + rs(4)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];
M7 = [1 0 0 (rs(2) + rs(4) + rs(6)); 0 1 0 0; 0 0 1 (rs(1) - rs(3) + rs(5)); 0 0 0 1];

% %Store the joint orientations in a multidimensional matrix.
[M(:, :, 1), M(:, :, 2), M(:, :, 3), M(:, :, 4), M(:, :, 5), M(:, :, 6)] = deal(M2, M3, M4, M5, M6, M7);

%Define the screw axes.
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -rs(1) 0 0]';
S3 = [1 0 0 0 rs(1) 0]';
S4 = [0 1 0 (rs(3) - rs(1)) 0 rs(2)]';
S5 = [1 0 0 0 (rs(1) - rs(3)) 0]';
S6 = [0 1 0 (rs(3) - rs(1) - rs(5)) 0 (rs(2) + rs(4))]';

%Store the screw axes in a matrix.
S = [S1 S2 S3 S4 S5 S6];

%Compute the trajectory of each link point based on the link lengths and
%motor angles.
[ ps1, ps2, ps3, ps4, ps5, ps6  ] = GetTrajectory2( S, M, thetas );

%% Animate the Trajectories.

%Define the markersize.
mrksz = 10;

%Define a vector to store whether the link lengths are changing.
bLengthCheck = -1*ones(1, size(ps6, 2));

%Create and format a plot for the trajectories.
figure, hold on, grid on
PlotWorkspacePoints( [], WBs, mrksz )

%Iterate through all of the link point positions.
for k = 1:size(ps6, 2)
    
    %Explicity define link point position variables for animation.
    [x1, y1, z1] = deal( ps1(1, k), ps1(2, k), ps1(3, k) );
    [x2, y2, z2] = deal( ps2(1, k), ps2(2, k), ps2(3, k) );
    [x3, y3, z3] = deal( ps3(1, k), ps3(2, k), ps3(3, k) );
    [x4, y4, z4] = deal( ps4(1, k), ps4(2, k), ps4(3, k) );
    [x5, y5, z5] = deal( ps5(1, k), ps5(2, k), ps5(3, k) );
    [x6, y6, z6] = deal( ps6(1, k), ps6(2, k), ps6(3, k) );
    
    %Create arrays of the link point positions.
    xs = [0 x1 x2 x3 x4 x5 x6];
    ys = [0 y1 y2 y3 y4 y5 y6];
    zs = [0 z1 z2 z3 z4 z5 z6];
    
    %Retrieve the position of the end effector.
    [xs6, ys6, zs6] = deal( ps6(1, 1:k), ps6(2, 1:k), ps6(3, 1:k) );
    
    %Store the structure into a matrix of points.
    P = [xs; ys; zs];
    
    %Compute the vectors between the links.
    dP = diff(P, 1, 2);
    
    %Compute the length of each link.
    dPmag = vecnorm(dP);
    
    %Check whether the lengths are constant.
    bLengthCheck(k) = sum(round(dPmag, 8) == rs) == length(rs);
    
    %Animate the link positions.
    if k == 1                       %If this is the first iteration...
        
        %Plot the mechanism.
        h5 = plot3(xs6, ys6, zs6, '-', 'Markersize', 20, 'XDataSource', 'xs6', 'YDataSource', 'ys6', 'ZDataSource', 'zs6');
        
        %Plot the end effector.
        h = plot3(xs, ys, zs, '.-', 'Markersize', 20, 'XDataSource', 'xs', 'YDataSource', 'ys', 'ZDataSource', 'zs');
    else
        %Refresh the figure.
        refreshdata([h h5])
        drawnow
        
        %Wait a short amount of time.
%         pause(0.1)
    end
    
end

%Add the paths of all link points to the plot for reference after the
%animation is complete.
plot3(ps1(1, :), ps1(2, :), ps1(3, :), '-', 'Markersize', 20)
plot3(ps2(1, :), ps2(2, :), ps2(3, :), '-', 'Markersize', 20)
plot3(ps3(1, :), ps3(2, :), ps3(3, :), '-', 'Markersize', 20)
plot3(ps4(1, :), ps4(2, :), ps4(3, :), '-', 'Markersize', 20)
plot3(ps5(1, :), ps5(2, :), ps5(3, :), '-', 'Markersize', 20)
plot3(ps6(1, :), ps6(2, :), ps6(3, :), '-', 'Markersize', 20)
