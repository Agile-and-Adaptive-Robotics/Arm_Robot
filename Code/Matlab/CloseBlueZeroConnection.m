function CloseBlueZeroConnection(sim, clientID)

% Ensure that the previous commands had time to reach the client.
sim.simxGetPingTime(clientID);

% Close the Blue Zero connection.
sim.simxFinish(clientID);

% Delete the simulation variable.
sim.delete();

end

