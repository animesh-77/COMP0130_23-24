% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId; % (1,1) total count of vertices
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;

        % Q3b -- attribute that when set to be true calls for graph pruning
        % to take place 
        graphPrune = false;
        
    end
    
    methods(Access = public)

        % Create the localization system and start it up.
        function DrivebotSLAMSystem_obj = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            DrivebotSLAMSystem_obj = DrivebotSLAMSystem_obj@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            DrivebotSLAMSystem_obj.vehicleVertices = cell(1, DrivebotSLAMSystem_obj.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            DrivebotSLAMSystem_obj.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            DrivebotSLAMSystem_obj.processModelEdges = cell(1, DrivebotSLAMSystem_obj.INITIAL_CACHE_SIZE);
            DrivebotSLAMSystem_obj.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            DrivebotSLAMSystem_obj.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            DrivebotSLAMSystem_obj.recommendOptimizationPeriod = inf;
            
            DrivebotSLAMSystem_obj.removePredictionEdgesFromGraph = false;
            DrivebotSLAMSystem_obj.keepFirstPredictionEdge = false;
        end
        
        % Destroy the graph when we destroy the SLAM system.
        % Without this, MATLAB will crash whenever this object is destroyed.

        function delete(this)
            vertices = this.graph.vertices();

            for v = 1 : length(vertices)
                this.graph.removeVertex(vertices{v});
            end
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            % state and covariances
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId); % state vector 1 for each vertes
            P = zeros(this.NP, this.vehicleVertexId); % 
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            

            if (this.graphPrune)
                % if we have stated for graph pruning to take place, run
                % the function to do so 
                this.graphPruning()
            end


            % Remove the prediction edges if requested.
            if (this.removePredictionEdgesFromGraph)
                this.deleteVehiclePredictionEdges();
            end
            
            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end

        function setGraphPruning(DriveBotSLAMSystem_obj, toPruneOrNot)
            DriveBotSLAMSystem_obj.graphPrune = toPruneOrNot;
        end 
    end
    
    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(DriveBotSLAMSystem_obj, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            DriveBotSLAMSystem_obj.currentVehicleVertex = drivebot.graph.VehicleStateVertex(DriveBotSLAMSystem_obj.currentTime);
            DriveBotSLAMSystem_obj.currentVehicleVertex.setEstimate(event.data);
            DriveBotSLAMSystem_obj.graph.addVertex(DriveBotSLAMSystem_obj.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            DriveBotSLAMSystem_obj.vehicleVertexId = 1;
            DriveBotSLAMSystem_obj.vehicleVertices{DriveBotSLAMSystem_obj.vehicleVertexId} = DriveBotSLAMSystem_obj.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                DriveBotSLAMSystem_obj.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.graph.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(DriveBotSLAMSystem_obj.currentVehicleVertex);
                DriveBotSLAMSystem_obj.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(DriveBotSLAMSystem_obj, time, dT)

            % Create the next vehicle vertex and add it to the graph
            DriveBotSLAMSystem_obj.currentVehicleVertex = drivebot.graph.VehicleStateVertex(time);
            % DriveBotSLAMSystem_obj.graph.addVertex(DriveBotSLAMSystem_obj.currentVehicleVertex);

            % Create the edge
            processModelEdge = drivebot.graph.VehicleKinematicsEdge(dT);
            processModelEdge.setVertex(1, DriveBotSLAMSystem_obj.vehicleVertices{DriveBotSLAMSystem_obj.vehicleVertexId});
            processModelEdge.setVertex(2, DriveBotSLAMSystem_obj.currentVehicleVertex); 
            processModelEdge.setMeasurement(DriveBotSLAMSystem_obj.u);
            processModelEdge.setInformation(inv(DriveBotSLAMSystem_obj.uCov));
            processModelEdge.initialize();
            
            % adding edge to graph
            DriveBotSLAMSystem_obj.graph.addEdge(processModelEdge);

            % Add the prediciton to the new vertex
            DriveBotSLAMSystem_obj.currentVehicleVertex.setEstimate(processModelEdge.vertex(2).estimate());

            % adding the current vehicle vertex to the graph
            DriveBotSLAMSystem_obj.graph.addVertex(DriveBotSLAMSystem_obj.currentVehicleVertex);

            DriveBotSLAMSystem_obj.numProcessModelEdges = DriveBotSLAMSystem_obj.numProcessModelEdges + 1;
            DriveBotSLAMSystem_obj.processModelEdges{DriveBotSLAMSystem_obj.numProcessModelEdges} = processModelEdge;
            
            %warning('drivebotslam:handlepredicttotime:unimplemented', ...
            %    'Implement the rest of this method for Q1b.');
            
            % Bump the indices
            DriveBotSLAMSystem_obj.vehicleVertexId = DriveBotSLAMSystem_obj.vehicleVertexId + 1;
            DriveBotSLAMSystem_obj.vehicleVertices{DriveBotSLAMSystem_obj.vehicleVertexId} = DriveBotSLAMSystem_obj.currentVehicleVertex;


        end
        
        function handleGPSObservationEvent(DriveBotSLAMSystem_obj, event)

            % Q1d:
            % Create a GPS measurement edge and add it to the graph
            % warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
            %     'Implement the rest of this method for Q1c.');
            omegaR = inv(event.covariance);
            k = DriveBotSLAMSystem_obj.vehicleVertexId;
            
            % Create the measurement edge
            GPSMeasurementEdge_obj = drivebot.graph.GPSMeasurementEdge(...
                DriveBotSLAMSystem_obj.configuration.gpsPositionOffset);

            % Link it so that it connects to the vertex we want to estimate
            GPSMeasurementEdge_obj.setVertex(1, DriveBotSLAMSystem_obj.vehicleVertices{k});

            % Set the measurement value and the measurement covariance
            GPSMeasurementEdge_obj.setMeasurement(event.data);
            GPSMeasurementEdge_obj.setInformation(omegaR);
  
            % Add the edge to the graph
            DriveBotSLAMSystem_obj.graph.addEdge(GPSMeasurementEdge_obj);
            
        end
        
        function handleCompassObservationEvent(DriveBotSLAMSystem_obj, event)
            
            % Q1c
            % Create a compass measurement edge and add it to the graph
            compassMeasurementEdge_obj = drivebot.graph.CompassMeasurementEdge(...
                DriveBotSLAMSystem_obj.configuration.compassAngularOffset);
            compassMeasurementEdge_obj.setVertex(1, DriveBotSLAMSystem_obj.currentVehicleVertex);
            A= event.data;
            if A > pi || A< -pi
                error("Need to use normalise theta")
            end
            compassMeasurementEdge_obj.setMeasurement(A); % added normalise_theta() here
            compassMeasurementEdge_obj.setInformation(inv(event.covariance));
            DriveBotSLAMSystem_obj.graph.addEdge(compassMeasurementEdge_obj);
        end
        
        function handleLandmarkObservationEvent(DriveBotSLAMSystem_obj, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = DriveBotSLAMSystem_obj.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);

                % Q2b:
                % Complete the implementation
                % warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
                %     'Implement the rest of this method for Q2b.');


                % Add the measurement edge
                landmarkRangeBearingEdge = drivebot.graph.LandmarkRangeBearingEdge();
                landmarkRangeBearingEdge.setVertex(1, DriveBotSLAMSystem_obj.vehicleVertices{DriveBotSLAMSystem_obj.vehicleVertexId});
                landmarkRangeBearingEdge.setVertex(2, landmarkVertex);
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));

                if (newVertexCreated == true)
                    landmarkRangeBearingEdge.initialize();
                end
                
                
                DriveBotSLAMSystem_obj.graph.addEdge(landmarkRangeBearingEdge);
            end
        end
        
        function deleteVehiclePredictionEdges(DriveBotSLAMSystem_obj)

            % Q3a:            
            % warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
            %     'Implement the rest of this method for Q3a.');
             

            % find the number of edges 
            edges = DriveBotSLAMSystem_obj.graph.edges();
            numOfEdges = length(edges);
            fprintf('Initial number of edges %d \n',numOfEdges)

            % initialise the edge count 
            count = 0;
            deletedEdges = 0;

            % loop through all the edges and delete them all 
            for i = 1:numOfEdges
                
                % check if it's a prediction edge 
                % if class(edges{i}) == "drivebot.graph.VehicleKinematicsEdge"
                if isa(edges{i}, 'drivebot.graph.VehicleKinematicsEdge')
                    count = count + 1;

                    % check if we need to keep the first edge 
                    if (DriveBotSLAMSystem_obj.keepFirstPredictionEdge == true && count == 1)

                        % if we need to keep the first edge, move onto the
                        % next one without deleting the first                         
                        continue 
                        
                    else 
                       % otherwise, remove the current vehicle kinematics edge
                        DriveBotSLAMSystem_obj.graph.removeEdge(edges{i});
                        
                        deletedEdges = deletedEdges + 1;
                        
                    end 
                        
                end  
                
            end 

            fprintf('Number of deleted edges %d \n',deletedEdges)

            edges = DriveBotSLAMSystem_obj.graph.edges();
            numOfEdges = length(edges);
            fprintf('Final number of edges %d \n',numOfEdges)
        end
        
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)

            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.graph.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
        function graphPruning(this)            
            edges = this.graph.edges();
            numEdges = length(edges);
            fprintf("No. of edges before graph pruning %d \n", numEdges)
            
            vertices = this.graph.vertices();
            numVertices = length(vertices);  
            fprintf("No. of vertices before graph pruning %d \n", numVertices)
            
            obsEdgeCount = 0; 
            
            actualNumVertices = 0; 
            removedEdges = 0;
            
            for i = 1:numVertices % loop through all vertices 
                
                vertex = this.vehicleVertices{i};         
                
                if isa(vertex,"drivebot.graph.VehicleStateVertex")
                    
                    actualNumVertices = actualNumVertices + 1;
                    
                    vertexEdges = vertex.edges();
                    
                    numVertexEdges = length(vertexEdges);
                    
                    for j = 1:numVertexEdges 
                        
                        if isa(vertexEdges{j}, "drivebot.graph.LandmarkRangeBearingEdge")
                            obsEdgeCount = obsEdgeCount + 1;
                            
                        end 
                    end
                end 
                                   
            end
            
            actualNumVertices;
            
            avgObsEdges = round(obsEdgeCount / actualNumVertices); 
         
            for i = 1:numVertices 
                obsEdgeCount = 0;      

                vertex = this.vehicleVertices{i};
                if isa(vertex, 'drivebot.graph.VehicleStateVertex')
                    
                    % find cell of all edges for this vertex 
                    vertexEdges = vertex.edges();
                    numVertexEdges = length(vertexEdges);
                        
                    % loop through all edges and count the number of
                    % observation edges 
                     for j = 1:numVertexEdges 
                        thisEdge = vertexEdges{j};

                        if isa(thisEdge, 'drivebot.graph.LandmarkRangeBearingEdge')
                            
                            obsEdgeCount = obsEdgeCount + 1; 
                       
                        end 
                     end
                     
                     if obsEdgeCount < avgObsEdges && (i ~= 1) && (i ~=2)
                         
                         for k = 1:numVertexEdges 
                            thisEdge = vertexEdges{k};
                            this.graph.removeEdge(thisEdge);

                            % increment number of removed edges 
                            removedEdges = removedEdges + 1;

                        end 
                         
                         
                     end 
                     
                end 
            end
            
        edges = this.graph.edges();
        numEdges = length(edges);
        fprintf("No. of edges before graph pruning %d \n", numEdges)
        
        vertices = this.graph.vertices();
        numVertices = length(vertices);  
        fprintf("No. of vertices before graph pruning %d \n", numVertices)


        end
    end
end
