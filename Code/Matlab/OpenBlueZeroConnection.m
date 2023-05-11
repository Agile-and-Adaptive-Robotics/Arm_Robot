function [sim, clientID] = OpenBlueZeroConnection()


sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
% clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);  %Start up Matlab's connection with BlueZero
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);  %Start up Matlab's connection with BlueZero


end

