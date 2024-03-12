% This script runs Q3(b)

% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% SLAM is enabled, GPS disabled
configuration_obj.enableGPS = false;
configuration_obj.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration_obj.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration_obj.laserDetectionRange = 30;

% Set up the simulator
DriveBotSimulator_obj = drivebot.DriveBotSimulator(configuration_obj, 'q3_a');

% Create the localization system
DriveBotSLAMSystem_obj = drivebot.DriveBotSLAMSystem(configuration_obj);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
DriveBotSLAMSystem_obj.setValidateGraph(true);

% Optimize every 500 timesteps to get a picture of the situation evolves
DriveBotSLAMSystem_obj.setRecommendOptimizationPeriod(500);

% Q3b -- set this attribute to true so that we can run graph pruning after
% graph has been generated
DriveBotSLAMSystem_obj.setGraphPruning(true);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
% retain_first= true;
% drivebotSLAMSystem.setRemovePredictionEdges(true, retain_first);


% Run the main loop and correct results
results = minislam.mainLoop(DriveBotSimulator_obj, DriveBotSLAMSystem_obj);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
directory= 'Images/q3_b';
if ~exist(directory, 'dir')
    mkdir(directory);
end


% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on
title('Optimization times')
xlabel('Timestep')
ylabel('Optimisation Time (sec)')
saveas(gcf, fullfile(directory, 'Optimisation_times.svg'), 'svg');

% Plot the error curves
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

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
saveas(gcf, fullfile(directory, 'Vehicle_covariances.svg'), 'svg');

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
% wrap theta in [-pi, pi]
errors = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
errors(:,3) = g2o.stuff.normalize_thetas(errors(:,3));
plot(errors)
hold on
legend('x error', 'y error', '\psi error')
title('Errors')
xlabel('Timestep')
ylabel('error')
saveas(gcf, fullfile(directory, 'errors.svg'), 'svg');


% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
%warning('q3_a:unimplemented', 'Compute the diagnostics required for Q3a.');
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on
title('Chi 2 values')
xlabel('Timestep')
ylabel('Chi2 Value')
saveas(gcf, fullfile(directory, 'Chi2.svg'), 'svg');


