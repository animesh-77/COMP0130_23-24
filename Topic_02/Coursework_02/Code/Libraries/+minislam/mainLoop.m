function results = mainLoop(DriveBotSimulator_obj, drivebotSLAMSystem_obj)

% This function runs the main loop for the MiniSLAM system. It sets up the
% event generator and the localization systems. It sets up the graphics. It
% runs the main loop and collects results.

% Set up the graphics for output
graphicalOutput = minislam.graphics.GraphicalOutput();
graphicalOutput.initialize(DriveBotSimulator_obj, drivebotSLAMSystem_obj);

% Helper to make passing arguments easier
if (iscell(drivebotSLAMSystem_obj) == false)
    drivebotSLAMSystem_obj = {drivebotSLAMSystem_obj};
end

% Get the number of localization systems
numLocalizationSystems = length(drivebotSLAMSystem_obj);

% Allocate the results structure
results = cell(numLocalizationSystems, 1);
for l = 1 : numLocalizationSystems
    results{l} = minislam.Results();
end

% Start the event generator
DriveBotSimulator_obj.start();

storeCount = 1;

% Loop until we terminate
while (DriveBotSimulator_obj.keepRunning() == true)
    
    % Get the step number
    storeCount = DriveBotSimulator_obj.stepCount() + 1;
    
    % Print out
    if (rem(storeCount, 100) == 0)
        disp(num2str(storeCount))
    end
    
    % Get the events and generate the events
    events = DriveBotSimulator_obj.events();
    
    % Log the ground truth
    groundTruthState = DriveBotSimulator_obj.groundTruth(false);
    
    % Iterate over all the localization systems and process
    for l = 1 : numLocalizationSystems
        
        % Handle the event for each localization system
        localizationSystem = drivebotSLAMSystem_obj{l};    
        localizationSystem.processEvents(events);
        runOptimizer = localizationSystem.recommendOptimization();
    
        % If requested, run the optimizer and log the time required
        if (runOptimizer == true)
            tic
            chi2 = localizationSystem.optimize();
            results{l}.optimizationTimes(storeCount) = toc;
            results{l}.chi2Time = cat(1, results{l}.chi2Time, ...
                DriveBotSimulator_obj.time());
            results{l}.chi2History = cat(1, results{l}.chi2History, chi2);
        else
            results{l}.optimizationTimes(storeCount) = NaN;
        end
        
        % Store ground truth in each results structure
        results{l}.vehicleTrueStateTime(storeCount) = DriveBotSimulator_obj.time();
        results{l}.vehicleTrueStateHistory(:, storeCount) = groundTruthState.xTrue;
    end
    
    % Draw the graphics. Since this can be fairly slow, we currently only
    % run it when the optimizer is run
    if (runOptimizer == true)%
        graphicalOutput.update();
    end
    
    DriveBotSimulator_obj.step();
end

% Handle the end of the run. If the graph can be optimized,

for l = 1 : numLocalizationSystems
    % Get the localization system
    localizationSystem = drivebotSLAMSystem_obj{l};  
    % If recommend optimization was true, we optimized the graph before the
    % while loop ended and so there was nothing to do. Therfore, if
    % recomment Optimization() is false, the localization system might
    % contain additional vertices and edges which need optimizing. These
    % final results are written into the position given by storeCount.
    if (localizationSystem.recommendOptimization() == false)
        tic
        chi2 = localizationSystem.optimize(20);
        results{l}.optimizationTimes(storeCount) = toc;
        results{l}.chi2Time = cat(1, results{l}.chi2Time, ...
            DriveBotSimulator_obj.time());
        results{l}.chi2History = cat(1, results{l}.chi2History, chi2);
    end
    [T, X, P] = localizationSystem.platformEstimateHistory();
    results{l}.vehicleStateTime = T;
    results{l}.vehicleStateHistory = X;
    results{l}.vehicleCovarianceHistory = P;
end

% Make sure that the graphics output is updated to show the final state
graphicalOutput.update();

end