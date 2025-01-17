% This class implements the interface for a SLAM system

classdef SLAMSystem < ebe.core.EventBasedEstimator
    
    methods(Access = public)
        function obj = SLAMSystem(configuration)
            % Call the base class
            obj@ebe.core.EventBasedEstimator(configuration);
        end
    end

    methods(Access = public, Abstract)

        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = platformEstimate(this);
        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = platformEstimateHistory(this);
        
        % Get the current landmarks estimates.
        % TODO - change to something like a map structure?
        [x, P, landmarkIds] = landmarkEstimates(this);
                
    end


end
