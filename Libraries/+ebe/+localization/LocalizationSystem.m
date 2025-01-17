classdef LocalizationSystem < ebe.core.EventBasedEstimator

    methods(Access = public)
        function obj = LocalizationSystem(configuration)
            % Call the base class
            obj@ebe.core.EventBasedEstimator(configuration);
        end
    end

    methods(Access = public, Abstract)

        [T, X, PX] = estimateHistory(obj);

        [x, P] = computeXP(obj);

    end

end