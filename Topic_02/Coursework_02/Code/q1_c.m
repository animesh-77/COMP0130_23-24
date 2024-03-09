% This script runs Q1(c)

% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% Q1c: Set the configuration to enable the compass

% Set the compass angular offset. DO NOT change this value.
configuration_obj.compassAngularOffset=0.75*pi;
configuration_obj.enableCompass= true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration_obj.perturbWithNoise = true;

% Set up the simulator
DriveBotSimulator_obj = drivebot.DriveBotSimulator(configuration_obj, 'q1_c');

% Create the localization system
DrivebotSLAMSystem_obj = drivebot.DriveBotSLAMSystem(configuration_obj);

% Force the optimizer to run with this frequency. This lets you see what's
% happening in greater detail, but slows everything down.
DrivebotSLAMSystem_obj.setRecommendOptimizationPeriod(20);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
DrivebotSLAMSystem_obj.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(DriveBotSimulator_obj, DrivebotSLAMSystem_obj);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
directory= 'Images/q1_c';
if ~exist(directory, 'dir')
    mkdir(directory);
end

% Plot optimisation times
% If DrivebotSLAMSystem_obj.setRecommendOptimizationPeriod() not used then
% this will be a blank graph
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
saveas(gcf, fullfile(directory, 'Optimisation_times.svg'), 'svg');
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
saveas(gcf, fullfile(directory, 'Errors.svg'), 'svg');
legend('x', 'y', 'psi')
hold on

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('x', 'y', 'psi')
saveas(gcf, fullfile(directory, 'Vehicle_covariances.svg'), 'svg');
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x', 'y', 'psi')
saveas(gcf, fullfile(directory, 'Errors_2.svg'), 'svg');
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, log(results{1}.chi2History))
legend('x', 'y', 'psi')
saveas(gcf, fullfile(directory, 'Chi2.svg'), 'svg');
hold on

