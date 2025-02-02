classdef SLAMSystem < ebe.core.EventBasedEstimator
    % SLAMSystem summary of SLAMSystem
    %
    % A SLAM system is one which builds a map of the environment and
    % estimates the platform state at the same time. The main thing this
    % class provides is an interface for extracting the estimates.
    
    methods(Access = public)
        function obj = SLAMSystem(configuration)
            % SLAMSystem Constructor for SLAMSystem
            %
            % Syntax:
            %   slamSystem = SLAMSystem(config)
            %
            % Description:
            %   Constructs a SLAM system using the provided configuration.
            %   Note this is an abstract class and cannot be instantiated
            %   directly. 
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %
            % Outputs:
            %   slamSystem - (handle)
            %       An instance of a SLAMSystem
            obj@ebe.core.EventBasedEstimator(configuration);
        end
    end

    methods(Access = public, Abstract)

        [x, P] = platformEstimate(obj);
            % PLATFORMESTIMATE Return the current mean and covariance of the
            % platform estimate
            %
            % Syntax:
            %   [x, P] = slamSystem.platformEstimate()
            %
            % Description:
            %   Return the current mean and covariance estimate of the
            %   platform. Note that the covariance estimate is the full
            %   covariance matrix
            %
            % Outputs:
            %   X - (nx1 vector)
            %       The estimated state at the current time
            %   PX - (nxn double psd matrix)
            %       The covariance matrix of the estimate

        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = platformEstimateHistory(this);
        
        [x, P, landmarkIds] = landmarkEstimates(this);
                
    end


end
