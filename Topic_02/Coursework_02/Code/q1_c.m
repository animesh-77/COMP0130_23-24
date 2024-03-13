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
saveas(gcf, fullfile(directory, 'Errors.svg'), 'svg');

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
for log_chi= [true, false]
    minislam.graphics.FigureManager.getFigure('chi2 values');
    clf
    
    if log_chi
        plot(results{1}.chi2Time, log(results{1}.chi2History)) % notice the log
        title('log(Chi2) values')
        ylabel('log(Chi2) Value')
        xlabel('Timestep')
        saveas(gcf, fullfile(directory, 'log_Chi2.svg'), 'svg');
    else
        plot(results{1}.chi2Time, results{1}.chi2History) % notice the log
        title('Chi2 values')
        ylabel('Chi2 Value')
        xlabel('Timestep')
        saveas(gcf, fullfile(directory, 'Chi2.svg'), 'svg');
    end
end

