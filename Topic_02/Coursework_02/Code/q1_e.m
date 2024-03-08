% This script runs Q1(e)

% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration_obj.enableGPS = true;

% Set to true for part ii
configuration_obj.enableCompass = true;

% Set up the simulator
DriveBotSimulator_obj = drivebot.DriveBotSimulator(configuration_obj, 'q1_e');

% Create the localization system
DriveBotSLAMSystem_obj = drivebot.DriveBotSLAMSystem(configuration_obj);

% Q1(e)i:
% Use the method "setRecommendOptimizationPeriod" in DriveBotSLAMSystem
% to control the rate at which the optimizer runs
disp(DriveBotSLAMSystem_obj.recommendOptimization)
DriveBotSLAMSystem_obj.setRecommendOptimizationPeriod(1);
disp(DriveBotSLAMSystem_obj.recommendOptimization)



% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
DriveBotSLAMSystem_obj.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(DriveBotSimulator_obj, DriveBotSLAMSystem_obj);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on


