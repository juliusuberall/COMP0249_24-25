classdef Event < handle

    % Event
    %
    % This is the base class for all events which propagate data
    %
    % Properties:
    %   time - The time of the event real number
    %   type - A string which specifies the event type
    %   data - The data (optional) typically this will be the
    %          numerical value of the observation
    %   covariance - The covariance matrix

    properties(GetAccess = public, SetAccess = protected)
        
        % The time of the event.
        time; 
        
        % The type of the event.
        type;
        
        % The event data
        data;
        
        % The noise on the event data
        covariance;

        % Any additional information
        info;
    end

    properties(Access = public)
    
        % Simulator timestep number; used for debugging
        eventGeneratorStepNumber;
    end

    methods(Access = public)
        function obj = Event(time, type, data, covariance, info)
            
            % Copy over the common values
            obj.time = time;
            obj.type = type;

            % Return if no data / covariance
            if (nargin < 3)
                return;
            end

            % Copy the event data
            obj.data = data;

            % Return if no covariance
            if (nargin < 4)
                return
            end
            
            % If the noise is a vector, assume that it encodes a diagonal covariance
            % matrix. Therefore, reshape into a matrix. If the covariance
            % is not a vector, check that it's a square matrix
            if ((size(covariance, 1) == 1) || (size(covariance, 2) == 1))
                covariance = diag(covariance);
            else
                r = size(covariance, 1);
                c = size(covariance, 2);
                assert(r == c, ...
                    'minislam:event:covariancenotsquare', ...
                    'The covariance is non-square matrix of size (%d,%d) %s', ...
                    r, c);
            end

            % Check the covariance is positive semidefinite
            assert(min(eig(covariance)) >= 0, ...
                    'minislam:event:covariancenotpsd', ...
                    'The covariance is non-positive semidefinite');

            obj.covariance = covariance;

            if (nargin == 5)
                obj.info = info;
            end
        end
    end
end
