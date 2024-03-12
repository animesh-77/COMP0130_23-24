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
last_step= 1207; % last timestep before loop closure
configuration_obj.maximumStepNumber = last_step;

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

% save the map before loop closure
minislam.graphics.FigureManager.getFigure('Simulator Output');
saveas(gcf, fullfile(directory, 'map_before.svg'), 'svg');



% Plot optimisation times.
% minislam.graphics.FigureManager.getFigure('Optimization times');
% clf
% plot(results{1}.optimizationTimes, '*')
% saveas(gcf, fullfile(directory, 'Optimisation_times_before.svg'), 'svg');
% hold on

% % Plot the error curves.
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
% saveas(gcf, fullfile(directory, 'Errors_before.svg'), 'svg');
% hold on

% Plot vehicle covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
veh_cov_vals= results{1}.vehicleCovarianceHistory' ;% (N,3) matrix
plot(veh_cov_vals)
hold on
title("Before loop closure")
text(last_step, veh_cov_vals(end,1), ['cov(x)', num2str(veh_cov_vals(end,1))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
text(last_step, veh_cov_vals(end,2), ['cov(y)', num2str(veh_cov_vals(end,2))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
text(last_step, veh_cov_vals(end,3), ['cov(psi)', num2str(veh_cov_vals(end,3))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
legend('x','y','psi')
legend('Location', 'northwest');
saveas(gcf, fullfile(directory, 'Vehicle_covariances_before.svg'), 'svg');


% Plot vehicle covariance.
minislam.graphics.FigureManager.getFigure('Landmark Covariances');
clf
land_cov_vals= results{1}.vehicleCovarianceHistory' ;% (N,2) matrix
plot(land_cov_vals)
hold on
text(last_step, land_cov_vals(end,1), ['cov(x)', num2str(land_cov_vals(end,1))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
text(last_step, land_cov_vals(end,2), ['cov(y)', num2str(land_cov_vals(end,2))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
legend('x','y','psi')
legend('Location', 'northwest');
saveas(gcf, fullfile(directory, 'landmark_covariances_before.svg'), 'svg');


% Plot errors.
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
% saveas(gcf, fullfile(directory, 'Errors2_before.svg'), 'svg');
% hold on

% Plot chi2 values.
% minislam.graphics.FigureManager.getFigure('chi2 values');
% clf
% plot(results{1}.chi2Time, results{1}.chi2History)
% saveas(gcf, fullfile(directory, 'Chi2_before.svg'), 'svg');
% hold on






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
configuration_obj.maximumStepNumber = last_step+1;

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


% save the map after loop closure
minislam.graphics.FigureManager.getFigure('Simulator Output');
saveas(gcf, fullfile(directory, 'map_after.svg'), 'svg');


% Plot optimisation times.
% minislam.graphics.FigureManager.getFigure('Optimization times');
% clf
% plot(results{1}.optimizationTimes, '*')
% saveas(gcf, fullfile(directory, 'Optimisation_times_after.svg'), 'svg');
% hold on

% Plot the error curves.
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
% saveas(gcf, fullfile(directory, 'Errors_after.svg'), 'svg');
% hold on

% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
veh_cov_vals= results{1}.vehicleCovarianceHistory';
plot(veh_cov_vals)
hold on
title("After loop closure")
text(last_step, veh_cov_vals(end,1), ['cov(x)', num2str(veh_cov_vals(end,1))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
text(last_step, veh_cov_vals(end,2), ['cov(y)', num2str(veh_cov_vals(end,2))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
text(last_step, veh_cov_vals(end,3), ['cov(psi)', num2str(veh_cov_vals(end,3))], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
legend('x','y','psi')
legend('Location', 'northwest');
saveas(gcf, fullfile(directory, 'Vehicle_covariances_after.svg'), 'svg');
hold on

% Plot errors.
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
% saveas(gcf, fullfile(directory, 'Errors2_after.svg'), 'svg');
% hold on

% Plot chi2 values.
% minislam.graphics.FigureManager.getFigure('chi2 values');
% clf
% plot(results{1}.chi2Time, results{1}.chi2History)
% saveas(gcf, fullfile(directory, 'Chi2_after.svg'), 'svg');
% hold on

