classdef EventGeneratorView < ebe.core.ConfigurableComponent

    properties(Access = protected)
        eventGenerator;
    end

    methods(Access = public)

        function obj = EventGeneratorView(config, eventGenerator)
            obj@ebe.core.ConfigurableComponent(config);
            obj.eventGenerator = eventGenerator;
        end
            
    end

    methods(Access = public, Abstract)
        visualize(obj, eventArray);
    end
end