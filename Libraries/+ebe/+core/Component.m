classdef Component < handle

    properties(Access = protected)

        componentName;

    end

    methods(Access = public, Sealed)
        
        function setName(obj, name)
            obj.componentName = name;
        end

        function n = name(obj)
            n = obj.componentName;
        end

    end

    methods(Access = public)

        function stop(obj)
            % For most components the default is do
            % nothing, which is why this isn't an
            % abstract method
        end

    end

    methods(Access = public, Abstract)

        start(obj);
    end

end