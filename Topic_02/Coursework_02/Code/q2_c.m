% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
directory= 'Images/q2_c';
if ~exist(directory, 'dir')
    mkdir(directory);
end

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
saveas(gcf, fullfile(directory, 'Optimisation_times.svg'), 'svg');
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
saveas(gcf, fullfile(directory, 'Errors.svg'), 'svg');
hold on

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
saveas(gcf, fullfile(directory, 'Vehicle_covariances.svg'), 'svg');
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
saveas(gcf, fullfile(directory, 'Errors_2.svg'), 'svg');
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
saveas(gcf, fullfile(directory, 'Chi2.svg'), 'svg');
hold on


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();


% Work out the number of vehicle poses and landmarks. 
numVertices = length(allVertices)
numEdges= length(allEdges)


numVehicleVertices = length(results{1}.vehicleStateHistory');
fprintf('Total number of vehicle poses stored: %.0d. \n', numVehicleVertices)


numLandmarkVertices = numVertices - numVehicleVertices;
fprintf('Total number of landmarks initalized: %.0d. \n', numLandmarkVertices)

numObsEdges = numEdges - numVehicleVertices -1;
numObsEdges_per_step = numObsEdges / numVehicleVertices;
fprintf(['Average number of observations made by a robot at' ...
    ' each timestep: %.2d. \n'], numObsEdges_per_step)


numObsEdges_per_landmark= numObsEdges/numLandmarkVertices;
fprintf(['Average number of observations made by a robot for' ...
    ' each landmark: %.2d. \n'], numObsEdges_per_landmark)

% landmarkObservationsPerVehicleVertex = 0;
% observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.
% warning('q2_c:unimplemented', ...
%         'Implement the rest of the graph query code for Q2c.')
