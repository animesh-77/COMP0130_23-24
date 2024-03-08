classdef CompassMeasurementEdge < g2o.core.BaseUnaryEdge
   
    % Q1c:
    % This implementation contains a bug. Identify the problem
    % and fix it as per the question.

    properties(Access = protected)
        
        compassAngularOffset;
        
    end
    
    methods(Access = public)
    
        function CompassMeasurementEdge_obj = CompassMeasurementEdge(compassAngularOffset)
            CompassMeasurementEdge_obj = CompassMeasurementEdge_obj@g2o.core.BaseUnaryEdge(1);
            CompassMeasurementEdge_obj.compassAngularOffset = compassAngularOffset;
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            this.errorZ = g2o.stuff.normalize_theta(...
                x(3) + this.compassAngularOffset - this.z); % normalize_theta implemented here
            % this.errorZ = x(3) + this.compassAngularOffset - this.z; % Without normalize_theta 
        end
        
        function linearizeOplus(this)
            this.J{1} = [0 0 1];
        end        
    end
end