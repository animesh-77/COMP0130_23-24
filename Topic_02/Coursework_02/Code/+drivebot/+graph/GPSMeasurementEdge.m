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
        
        M = [c -s;
            s c];

        GPSMeasurementEdge.errorZ = GPSMeasurementEdge.z-...
            priorX(1:2)- M*GPSMeasurementEdge.xyOffset;
        end
        
        function linearizeOplus(GPSMeasurementEdge)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        priorX = GPSMeasurementEdge.edgeVertices{1}.x;

        c = cos(priorX(3));
        s = sin(priorX(3));
        
        dx_g= GPSMeasurementEdge.xyOffset(1);
        dy_g= GPSMeasurementEdge.xyOffset(2);

        GPSMeasurementEdge.J{1}=...
            [-1 0 s*dx_g+c*dy_g;...
              0 -1 -c*dx_g+s*dy_g] ;
        end
    end
end
