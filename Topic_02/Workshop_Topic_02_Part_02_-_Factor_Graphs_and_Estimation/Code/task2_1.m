% Dynamic object example. This shows how to build a graph to estimate an
% object which has both position and velocity

import g2o.core.*;
import two_d_tracking.*;

% Some parameters
numberOfTimeSteps = 1000;
dT = 1;
sigmaR = 100;
sigmaQ = 0.01;

% Work out the state transition equations
F0=[1 dT; 0 1]; % (2,2) submatrix
Q0=[dT^3/3 dT^2/2;dT^2/2 dT] * sigmaQ; % (2,2) submatrix

F = [F0 zeros(2); zeros(2) F0]; % (4,4) matrix
Q = [Q0 zeros(2); zeros(2) Q0]; % (4,4) matrix
R = eye(2) * sigmaR;

H = [1 0 0 0;
    0 0 1 0];% (2,4) 
% takes trueX and extracts x_pos and y_pos

% Work out the information matrices
omegaR = inv(R);
omegaQ = inv(Q);

% Ground truth array
trueX = zeros(4, numberOfTimeSteps); % (4,N)
z = zeros(2, numberOfTimeSteps); % (2,N) GPS pos




% First timestep
trueX(2, 1) = 0.1; % speed_x
trueX(4, 1) = -0.1; % speed_y
% x_pos and y_pos remain 0
z(:, 1) = H * trueX(:, 1) + sqrtm(sigmaR) * randn(2, 1); % (2,N) GPS pos

% Now predict the subsequent steps
for k = 2 : numberOfTimeSteps
    trueX(:, k) = F * trueX(:, k - 1) + sqrtm(Q) * randn(4, 1);
    % (4,4)*(4, 1)+ (4, 1)
    % process model
    z(:, k) = H * trueX(:, k) + sqrtm(sigmaR) * randn(2, 1);
    % (2,4)*(4,1)+ (2,1)
    % x_pos,y_pos + Noise --> GPS pos
    % this is what we observe and will be stored in
    % ObjectMeasurementEdge.Measurement
end

% Create the graph
graph = SparseOptimizer();
algorithm = GaussNewtonOptimizationAlgorithm();
graph.setAlgorithm(algorithm);

% This array contains the set of vertices for the target state over time
v = cell(numberOfTimeSteps, 1); % cell array a new kind of data structure
% (n,1)

% Now create the vertices and edges

for n = 1 : numberOfTimeSteps
    
    % Create the first object state vertex
    v{n} = ObjectStateVertex(); % two_d_tracking already implemented
    % which itself is a subclass

    % Set the initial estimate.
    v{n}.setEstimate(zeros(4, 1));

    % Another way to do it would be to initialize from pairs of
    % measurements. For example:
    %if (n == 1)
    %    dz = z(:, 2) - z(:, 1);
    %else
    %    dz = z(:, n) - z(:, n-1);
    %end
    %dzdt = dz / dT;
    %v{n}.setEstimate([z(1, n); dzdt(1); z(2, n); dzdt(2)]);
    
    % Added the vertex to the graph.
    graph.addVertex(v{n});
    
    % If this isn't the first vertex, add the dynamics
    if (n > 1)
        processModelEdge = ObjectProcessModelEdge();
        processModelEdge.setVertex(1, v{n-1});
        processModelEdge.setVertex(2, v{n});
        processModelEdge.setMeasurement([0;0;0;0]);
        processModelEdge.setF(F);
        processModelEdge.setInformation(omegaQ);
        graph.addEdge(processModelEdge);
    end
    
    % if (n<450) || (n>500)
    % Create the measurement edge
    e = ObjectMeasurementEdge();
    
    % Link it so that it connects to the vertex we want to estimate
    e.setVertex(1, v{n});
    
    % Set the measurement value and the measurement covariance
    e.setMeasurement(z(:,n));
    e.setInformation(omegaR);
    
    % Add the edge to the graph; the graph now knows we have these edges
    % which need to be added
    graph.addEdge(e);
    % end
end


% Graph construction complete









% Initialise the optimization. This is done here because it's a bit
% expensive and if we cache it, we can do it multiple times
graph.initializeOptimization();

% Create some output as we go
x = zeros(4, numberOfTimeSteps); % (4,N) matrix

% First copy the state values - these are the prior we set the graph to
for n = 1 : numberOfTimeSteps
    x(:, n) = v{n}.estimate();
end

% Plot the prior and the truth
figure(1)
clf

% Plot. Note that we capture the line handle. This is overkill in this
% case, but it's a useful habit to get into for labelling graphs.
gH(1)=plot(x(1, :), x(3,:), 'Marker', '.', 'MarkerSize', 20);
% x holds the prior values
% all PRIORS are at (0,0) so all points at origin
hold on
gH(2)=plot(trueX(1, :), trueX(3, :));
% Ground truth

% Optimize the graph
tic
% start of the time
graph.optimize(5000)
toc
% end of time, used to see elapsed time


disp("State estimates for t=1")
disp(v{1}.estimate())
disp("Dimensions of state t=1")
disp(v{1}.dimension)
disp("Edges of for this")
disp(v{1}.edges)


% Extract the optimized state estimate and plot
for n = 1 : numberOfTimeSteps
    x(:, n) = v{n}.estimate();
    % now x will hold the Posterior values
end
gH(3)=plot(x(1, :), x(3, :), 'LineWidth', 2);

gH(4)=plot(z(1,:),z(2,:));
% observed GPS values which are very noisy

% Generate the legend
legend(gH, {'Prior', 'Truth', 'Optimized', 'Observation'});
title([num2str(graph.chi2())])
drawnow

