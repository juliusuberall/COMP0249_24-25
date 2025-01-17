classdef EstimatorView < ebe.core.ConfigurableComponent

    properties(Access = protected)
        estimator;
    end

    methods(Access = public)

        function obj = EstimatorView(config, estimator)
            obj@ebe.core.ConfigurableComponent(config);
            obj.estimator = estimator;
        end
            
    end

    methods(Access = public, Abstract)
        visualize(obj, eventArray);
    end
end