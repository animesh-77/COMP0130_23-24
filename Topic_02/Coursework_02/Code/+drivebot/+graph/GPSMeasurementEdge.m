classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(VehicleKinematicsEdge_obj)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:computeerror:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        priorX = VehicleKinematicsEdge_obj.edgeVertices{1}.x;

        c = cos(priorX(3));
        s = sin(priorX(3));
        
        M = [c -s;
            s c];

        VehicleKinematicsEdge_obj.errorZ = VehicleKinematicsEdge_obj.z-...
            priorX(1:2)- M*VehicleKinematicsEdge_obj.xyOffset;
        end
        
        function linearizeOplus(VehicleKinematicsEdge_obj)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        priorX = VehicleKinematicsEdge_obj.edgeVertices{1}.x;

        c = cos(priorX(3));
        s = sin(priorX(3));
        
        dx_g= VehicleKinematicsEdge_obj.xyOffset(1);
        dy_g= VehicleKinematicsEdge_obj.xyOffset(2);

        VehicleKinematicsEdge_obj.J{1}=...
            [-1 0 s*dx_g+c*dy_g;...
              0 -1 c*dx_g+s*dy_g] ;
        end
    end
end
