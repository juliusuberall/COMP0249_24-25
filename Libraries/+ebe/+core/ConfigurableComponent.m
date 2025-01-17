classdef ConfigurableComponent < ebe.core.Component

    properties(Access = protected)

        config;

    end

    methods(Access = public)

        function obj = ConfigurableComponent(config)
            obj.config = config;
        end
    end

end