% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
% 
% M = [cos(theta) -sin(theta) 0; 
%      sin(theta) cos(theta) 0;
%       0 0 1];
% 
% The process model has the form:
% 
% x = x + M * [vx;vy;theta]
% 
% where vx, vy and vtheta are the velocities.
% 
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
    % properties(Access = public) % we change it so we can see it during
    % debugging
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge(dT)
            assert(dT >= 0);
            this = this@g2o.core.BaseBinaryEdge(3);            
            this.dT = dT;
        end
       
        function initialize(this)
            
                        
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;

            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));

        end
        
        function computeError(VehicleKinematicsEdge_obj)
    
            % Q1b:
            % Complete implementation
            % warning('vehiclekinematicsedge:computeerror:unimplemented', ...
            %         'Implement the rest of this method for Q1b.');
            % Rotation matrix from prior state
            priorX = VehicleKinematicsEdge_obj.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            Mi =  [c s 0;
                    -s c 0;
                    0 0 1]; % Inverse of M = M.T
            Mi= Mi /VehicleKinematicsEdge_obj.dT;

            dx = VehicleKinematicsEdge_obj.edgeVertices{2}.x - priorX; % Difference between two consequent states
            dx(3) = g2o.stuff.normalize_theta(dx(3)); % map in the range -pi to pi.
            
            VehicleKinematicsEdge_obj.errorZ = Mi  * (dx) - VehicleKinematicsEdge_obj.z;

        end
        
        % Compute the Jacobians
        function linearizeOplus(VehicleKinematicsEdge_obj)

            % Q1b:
            % Complete implementation
            % warning('vehiclekinematicsedge:linearizeoplus:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');
            priorX = VehicleKinematicsEdge_obj.edgeVertices{1}.x;
            c = cos(priorX(3));
            s = sin(priorX(3));
            dx = VehicleKinematicsEdge_obj.edgeVertices{2}.x - priorX; % this is u
            Mi = [c s 0;
                -s c 0;
                0 0 1];

            Mi= Mi/VehicleKinematicsEdge_obj.dT;


            VehicleKinematicsEdge_obj.J{2} = Mi; % the Jacobian is inverse of M/dT


            VehicleKinematicsEdge_obj.J{1}(1, 1) = - c/VehicleKinematicsEdge_obj.dT ; % towards each state
            VehicleKinematicsEdge_obj.J{1}(1, 2) = - s/VehicleKinematicsEdge_obj.dT;
            VehicleKinematicsEdge_obj.J{1}(1, 3) = (-dx(1) * s + dx(2) * c)/VehicleKinematicsEdge_obj.dT;
            VehicleKinematicsEdge_obj.J{1}(2, 1) = s/VehicleKinematicsEdge_obj.dT;
            VehicleKinematicsEdge_obj.J{1}(2, 2) = - c/VehicleKinematicsEdge_obj.dT;
            VehicleKinematicsEdge_obj.J{1}(2, 3) = (-dx(1) * c - dx(2) * s)/VehicleKinematicsEdge_obj.dT;
            VehicleKinematicsEdge_obj.J{1}(3, 3) = -1/VehicleKinematicsEdge_obj.dT;

        end
    end    
end