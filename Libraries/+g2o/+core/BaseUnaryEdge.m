% This class specializes a BaseEdge to the case that it plugs into just a
% single vertex.

classdef BaseUnaryEdge < g2o.core.BaseEdge

    methods(Access = protected)
   
        function obj = BaseUnaryEdge(measurementDimension)
            obj = obj@g2o.core.BaseEdge(1, measurementDimension);
        end
    end
end
