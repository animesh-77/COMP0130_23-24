% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % this is an object of class LandmarkRangeBearingEdge
            % Q2b:
            % Complete implementation
            % warning('landmarkrangebearingedge:initialize:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');
            vehicle_pos = this.edgeVertices{1}.x; % (3,1) matrix
            % range_bear= this.z; % (2,1) matrix range and bearing
            landmark_pos = zeros(2, 1);
            landmark_pos(1) = vehicle_pos(1) + this.z(1) * cos(this.z(2) + vehicle_pos(3));
            landmark_pos(2) = vehicle_pos(2) + this.z(1) * sin(this.z(2) + vehicle_pos(3));
            this.edgeVertices{2}.setEstimate(landmark_pos);
        end
        
        function computeError(this)

            % Q2b:
            % Complete implementation
            % warning('landmarkrangebearingedge:computeerror:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');
            x = this.edgeVertices{1}.x; %(3,1) matrix
            landmark = this.edgeVertices{2}.estimate(); % (2,1) matrix
            dx = landmark(1:2) - x(1:2);
            r= norm(dx);

            this.errorZ(1) = this.z(1) - r;
            this.errorZ(2) = g2o.stuff.normalize_theta(this.z(2) - atan2(dx(2), dx(1)) + x(3));
            

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Complete implementation
            % warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            dx = landmark(1:2) - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [dx(1)/r dx(2)/r 0;
                -dx(2)/r^2 dx(1)/r^2 1];
            this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end