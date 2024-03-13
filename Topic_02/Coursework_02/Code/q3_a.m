% This script runs Q3(a)
% sometimes it just fails. just re-run it

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation as it evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
retain_first= true;
drivebotSLAMSystem.setRemovePredictionEdges(true, retain_first);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.
if retain_first == true
    directory= 'Images/q3_a/retain_first';
else
    directory= 'Images/q3_a/no_retain';
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


% Plot chi2 values.
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

