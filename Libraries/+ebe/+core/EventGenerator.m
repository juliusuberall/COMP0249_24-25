% This class generates a stream of events. It could, for example, run a
% simulator, or could (eventually) be a wrapper on ROS.

classdef EventGenerator < ebe.core.ConfigurableComponent
    
    properties(Access = protected)

        % The step number
        stepNumber;

        % The most recent events
        outgoingEvents;

        % 
        outgoingEventsDispatched;
    end
    
    methods(Access = public)
        
        % Construct the object
        function obj = EventGenerator(config)
            obj = obj@ebe.core.ConfigurableComponent(config);
        end

        function start(obj)            
            % Set the current step number to zero
            obj.stepNumber = 0;

            % Set up new event queue for outgoing events
            obj.outgoingEvents = ebe.core.detail.OrderedEventQueue();
            obj.outgoingEventsDispatched = false;
        end

    end

    methods(Access = public, Sealed)
        
        function nextEvents = events(obj)
            nextEvents = obj.outgoingEvents.events();
            obj.outgoingEventsDispatched = true;
        end

        function clearOutgoingEvents(obj)
            obj.outgoingEvents.clear();
        end
        
        % Get the step number; this starts at 0 and advances by 1 after
        % each call to step.
        
        function stepNumber = stepCount(obj)
            stepNumber = obj.stepNumber;
        end
    end

    methods(Access = public, Abstract = true)

        % This method returns true as long as we should keep running
        carryOn = keepRunning(obj);

        % Step the event generator
        step(obj);
        
        % Get the current simulation time
        T = time(obj);
        
    end
end