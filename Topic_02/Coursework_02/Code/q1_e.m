% This script runs Q1(e)

% Create the configuration object.
configuration_obj = drivebot.SimulatorConfiguration();

% Since we are doing prediction and GPS, disable the SLAM sensor
configuration_obj.enableGPS = true;

% Set to true for part ii
enable_compass= true;
configuration_obj.enableCompass = enable_compass;

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
if enable_compass == true
   
    directory= 'Images/q1_e/compass_enabled';
else
    directory= 'Images/q1_e/no_compass';
end

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
saveas(gcf, fullfile(directory, 'errors.svg'), 'svg');


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


% Plot chi2 values
log_chi= false;
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
if log_chi
    plot(results{1}.chi2Time, log(results{1}.chi2History)) % notice the log
    title('log(Chi2) values')
    ylabel('log(Chi2) Value')
else
    plot(results{1}.chi2Time, results{1}.chi2History) % notice the log
    title('Chi2 values')
    ylabel('Chi2 Value')
hold on
end
xlabel('Timestep')
saveas(gcf, fullfile(directory, 'Chi2.svg'), 'svg');


