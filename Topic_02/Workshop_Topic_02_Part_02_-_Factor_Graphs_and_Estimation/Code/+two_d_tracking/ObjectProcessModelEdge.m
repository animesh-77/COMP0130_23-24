% This class models a linear process model. The process model is
%
% x(k+1)=F*x(k)+v(k)
%
% and v(k) is a Random Gaussian noise term
%
% The implementation is deliberately incomplete for Task 2.
% Rerranging, the subject of the formula is
% v(k) = F*x(k) - x(k+1)

classdef ObjectProcessModelEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        
        % The state transition matrix
        F;
        
    end
    
    methods(Access = public)
        function this = ObjectProcessModelEdge()
            this = this@g2o.core.BaseBinaryEdge(4);   
            
            % Create the state transtion matrix. Set it to NaN to trigger
            % errors if we don't assign a value to it.
            this.F = NaN(4, 4);            
        end
        
        function setF(this, F)
            this.F = F;
        end
       
        function computeError(this)
            % error('Complete this for task 2, part 1')
            this.errorZ = this.edgeVertices{2}.x - ...
                this.F * this.edgeVertices{1}.x;
        end
        
        function linearizeOplus(this)
            % error('Complete this for task 2, part 1')
            this.J{1} =  - this.F;
            this.J{2} = eye(4);
        end
    end
    
end