% This class generates a stream of events. It could, for example, run a
% simulator, or could (eventually) be a wrapper on ROS.

classdef EventBasedSimulator < ebe.core.EventGenerator
    
    properties(Access = protected)

        % The current time
        currentTime;

        % Queue which stores the next event generator which is queued up
        eventGeneratorQueue;

        % Scale which can be applied to noise. Set to 0
        % (no noise) or 1 (noise)
        noiseScale;

        % Flag; if set to false the simulator will terminate at the next
        % time step
        carryOnRunning;

        % Flag to show if debugging is enabled.
        debug;
        
        % Flag to show if the system has been initialized or not
        initialized;
    end
    
    methods(Access = public)
        
        % Construct the object
        function obj = EventBasedSimulator(config)
            obj = obj@ebe.core.EventGenerator(config);
        end

        function start(obj)

            start@ebe.core.EventGenerator(obj);

            % Set the start time to 0.
            obj.currentTime = 0;

            % Currently set to keep running
            obj.carryOnRunning = true;
            
            % Set that we need to initialize
            obj.initialized = false;

            % Create the queue for generating the events
            obj.eventGeneratorQueue = ebe.core.detail.EventGeneratorQueue();

            % Set the noise scale
            if (obj.config.perturbWithNoise == false)
                obj.noiseScale = 0;
            else
                obj.noiseScale = 1;
            end

        end
    end

    methods(Access = public, Sealed)
        
        % Get the current simulation time
        function T = time(obj)
            T = obj.currentTime;
        end

        % Return whether the simulator has finished
        function carryOn =  keepRunning(obj)
            carryOn = ((obj.carryOnRunning) && (obj.stepNumber <= obj.config.maximumStepNumber));
        end

        function step(obj)

            % Clear the event queue if the events() method was called
            % at least once before calling step().
            if (obj.outgoingEventsDispatched == true)
                obj.outgoingEvents.clear();
            end

            % Are there any pending events? If not, assume we are done
            if (obj.eventGeneratorQueue.empty() == true)
                obj.keepRunning = false;
                return
            end

            % Get the time and event generator that's next in the queue.
            [nextEventime, generateEvent] = obj.eventGeneratorQueue.pop();

            % This flag shows if we should increment to the next step
            incrementStepNumber = obj.initialized;

            % First see if we have to do time-based prediction step. We
            % only do this if (a) the system is initialized and (b) dT is
            % sufficiently big.

            % Figure out the delta from the current estimate time
            dT = nextEventime - obj.currentTime;
            dTOrig = dT;

            % Predict only if initialized
            if (obj.initialized == true)
    
                % Time should be non-negative otherwise there's a timestamp
                % skew.
                assert(dT >= 0);

                % Do not increment if really close to the last time
                if (dT <= obj.config.minDT)
                    incrementStepNumber = false;
                end

                % Predict forwards. Make sure that each call to prediction
                % is for a duration of no more than length maxDT. Note the
                % state predictor is run even if dT <= obj.config.minDT
                predictTime = obj.currentTime;
                while (dT > obj.config.minDT)
                    dTStep = min(dT, obj.config.maxDT);
                    predictTime = predictTime + dTStep;
                    obj.handlePredictForwards(dT, predictTime);
                    dT = dT - dTStep;
                end

                % Set the current time to now
                obj.currentTime = nextEventime;
            end

             if (incrementStepNumber == true)
                obj.stepNumber = obj.stepNumber + 1;
            end

            % Generate the next event
            generateEvent();

            % Store a step results. Note this catches the cases that either
            % (a) the suimulator was just initialized or (b) the prediction
            % interval is sufficiently long that we've bumped to a new
            % time step.
            if (obj.initialized == true) && (incrementStepNumber == true)
                % Handle any post-event activities
                obj.storeStepResults();
            end

            %fprintf('Simulator: %d %f (%f)\n',obj.stepNumber, obj.currentTime, dTOrig)

        end
    end

    methods(Access = protected, Abstract)


        % This handles the case that we predict to a given time
        handlePredictForwards(obj, dT, T);

        storeStepResults(obj);
    end
end