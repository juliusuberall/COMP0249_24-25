classdef ViewManager < ebe.core.ConfigurableComponent

    % Class to manage views contained in a single figure

    properties(Access = protected)

        % Handle for the figure
        figureHandle;

        % The set of views
        views;

    end

    methods(Access = public)

        function obj = ViewManager(config)
            obj@ebe.core.ConfigurableComponent(config);
            obj.views = {};
        end

        function addView(obj, view)
            obj.views{end+1} = view;
        end

        function start(obj)
            for v = 1 : numel(obj.views)
                obj.views{v}.start();
            end
        end

        function visualize(obj, eventArray)
            for v = 1 : numel(obj.views)
                obj.views{v}.visualize(eventArray);
            end
        end
    end
end