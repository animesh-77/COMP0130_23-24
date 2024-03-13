classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(GPSMeasurementEdge)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:computeerror:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        priorX = GPSMeasurementEdge.edgeVertices{1}.x;

        c = cos(priorX(3));
        s = sin(priorX(3));
        
        % Q1d building the matrix defined in Eq 1.2
        M = [c -s;
            s c];
        % Q1d refer to Eq 1.2 in report
        % We are treating the error as additve gaussian noise
        GPSMeasurementEdge.errorZ = GPSMeasurementEdge.z-...
            priorX(1:2)- M*GPSMeasurementEdge.xyOffset;
        end
        
        function linearizeOplus(GPSMeasurementEdge)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        priorX = GPSMeasurementEdge.edgeVertices{1}.x;
        
        % Q1d building the matrix defined in Eq 1.2
        c = cos(priorX(3));
        s = sin(priorX(3));
        % Q1d this is the difference bettween vehicle states in 2 vertices
        dx_g= GPSMeasurementEdge.xyOffset(1);
        dy_g= GPSMeasurementEdge.xyOffset(2);
        
        % Q1d refer to Eq 1.2 in report
        GPSMeasurementEdge.J{1}=...
            [-1 0 s*dx_g+c*dy_g;...
              0 -1 -c*dx_g+s*dy_g] ;
        end
    end
end
