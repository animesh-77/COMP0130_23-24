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
% If DrivebotSLAMSystem_obj.setRecommendOptimizationPeriod() not used then
% this will be a blank graph
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
saveas(gcf, fullfile(directory, 'Optimisation_times.png'), 'png');

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
% wrap theta in [-pi, pi]
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
hold on
legend('x error', 'y error', '\psi error')
legend('Location', 'best');
title('Errors')
xlabel('Timestep')
ylabel('error')
saveas(gcf, fullfile(directory, 'errors.png'), 'png');

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
legend('cov(x)', 'cov(y)', 'cov(\psi)')
legend('Location', 'best');
xlabel('Timestep')
title('Vehicle Covariances')
ylabel('covariance')
saveas(gcf, fullfile(directory, 'Vehicle_covariances.png'), 'png');


% Plot chi2 values
for log_chi= [true, false]
    minislam.graphics.FigureManager.getFigure('chi2 values');
    clf
    
    if log_chi
        plot(results{1}.chi2Time, log(results{1}.chi2History)) % notice the log
        title('log(Chi2) values')
        ylabel('log(Chi2) Value')
        xlabel('Timestep')
        saveas(gcf, fullfile(directory, 'log_Chi2.png'), 'png');
    else
        plot(results{1}.chi2Time, results{1}.chi2History) % notice the log
        title('Chi2 values')
        ylabel('Chi2 Value')
        xlabel('Timestep')
        saveas(gcf, fullfile(directory, 'Chi2.png'), 'png');
    end
end


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();


% Work out the number of vehicle poses and landmarks. 
numVertices = length(allVertices)
numEdges= length(allEdges)

numVehicleVertices= 0;
numLandmarkVertices= 0;
for i=1:numVertices
    current_vertex= allVertices{i};

    switch true
        case isa(current_vertex, 'drivebot.graph.VehicleStateVertex')
            numVehicleVertices = numVehicleVertices+ 1;
        case isa(current_vertex, 'drivebot.graph.LandmarkStateVertex')
            numLandmarkVertices = numLandmarkVertices+ 1;

        otherwise
            warning("New type of vertex!!! impossible")
            disp(class(current_vertex))
    end
end


fprintf('Total number of vehicle poses: %.0d. \n', numVehicleVertices)
fprintf('Total number of landmarks initalized: %.0d. \n', numLandmarkVertices)


numObsEdges= 0;
numKineEdges= 0;

for i=1:numEdges
    current_edge= allEdges{i};

    switch true

        case isa(current_edge, 'drivebot.graph.VehicleKinematicsEdge')
            numKineEdges = numKineEdges+ 1;
        case isa(current_edge, 'drivebot.graph.LandmarkRangeBearingEdge')
            numObsEdges = numObsEdges+ 1;
        otherwise
            warning("New type of Edge Initialisation ..." + ...
                "GPS or compasss!!! should be set off")
            disp(class(current_edge))
    end
end

numObsEdges_per_step = numObsEdges / numVehicleVertices;



fprintf(['Average number of observations made by a robot at' ...
    ' each timestep: %.2d. \n'], numObsEdges_per_step)


numObsEdges_per_landmark= numObsEdges/numLandmarkVertices;
fprintf(['Average number of observations made by a robot for' ...
    ' each landmark: %.2d. \n'], numObsEdges_per_landmark)

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.
% warning('q2_c:unimplemented', ...
%         'Implement the rest of the graph query code for Q2c.')
