% This script runs Q2(d)



% Before loop closure part
% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration_obj.enableLaser = true;

% For this part of the coursework, this should be set to false.
configuration_obj.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end
% here we go to the timestep just before loop closure
configuration_obj.maximumStepNumber = 1205;

% Set up the simulator
DriveBotSimulator_obj = drivebot.DriveBotSimulator(configuration_obj, 'q2_d');

% Create the localization system
DriveBotSLAMSystem_obj = drivebot.DriveBotSLAMSystem(configuration_obj);
DriveBotSLAMSystem_obj.setRecommendOptimizationPeriod(inf);

% Q2d:
% Explore the  timestep where the loop closure occurs, and get
% results just before and after the loop closure event
% error('q2_d:unimplemented', ...
%         'Analyse loop closure behaviour for Q2d.')

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
DriveBotSLAMSystem_obj.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(DriveBotSimulator_obj, DriveBotSLAMSystem_obj);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
directory= 'Images/q2_d';
if ~exist(directory, 'dir')
    mkdir(directory);
end

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
saveas(gcf, fullfile(directory, 'Optimisation_times_before.svg'), 'svg');
hold on

% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
saveas(gcf, fullfile(directory, 'Errors_before.svg'), 'svg');
hold on

% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
saveas(gcf, fullfile(directory, 'Vehicle_covariances_before.svg'), 'svg');
hold on

% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
saveas(gcf, fullfile(directory, 'Errors2_before.svg'), 'svg');
hold on

% Plot chi2 values.
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
saveas(gcf, fullfile(directory, 'Chi2_before.svg'), 'svg');
hold on






% After loop closure part
% Before loop closure part
% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration_obj.enableLaser = true;

% For this part of the coursework, this should be set to false.
configuration_obj.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end
% here we go to the timestep just after loop closure
configuration_obj.maximumStepNumber = 1208;

% Set up the simulator
DriveBotSimulator_obj = drivebot.DriveBotSimulator(configuration_obj, 'q2_d');

% Create the localization system
DriveBotSLAMSystem_obj = drivebot.DriveBotSLAMSystem(configuration_obj);
DriveBotSLAMSystem_obj.setRecommendOptimizationPeriod(inf);

% Q2d:
% Explore the  timestep where the loop closure occurs, and get
% results just before and after the loop closure event
% error('q2_d:unimplemented', ...
%         'Analyse loop closure behaviour for Q2d.')

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
DriveBotSLAMSystem_obj.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(DriveBotSimulator_obj, DriveBotSLAMSystem_obj);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
directory= 'Images/q2_d';
if ~exist(directory, 'dir')
    mkdir(directory);
end

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
saveas(gcf, fullfile(directory, 'Optimisation_times_after.svg'), 'svg');
hold on

% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
saveas(gcf, fullfile(directory, 'Errors_after.svg'), 'svg');
hold on

% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
saveas(gcf, fullfile(directory, 'Vehicle_covariances_after.svg'), 'svg');
hold on

% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
saveas(gcf, fullfile(directory, 'Errors2_after.svg'), 'svg');
hold on

% Plot chi2 values.
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
saveas(gcf, fullfile(directory, 'Chi2_after.svg'), 'svg');
hold on

