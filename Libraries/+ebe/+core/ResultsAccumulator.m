classdef ResultsAccumulator < ebe.core.Component

    properties(Access = protected)

        eventGenerator;

        estimators;

    end

    methods(Access = public)

        function setEventGeneratorAndEstimators(obj, eventGenerator, estimators)
            obj.eventGenerator = eventGenerator;
            obj.estimators = estimators;
        end
    end

    methods(Access = public, Abstract)

        collectResults(obj);

    end

end
