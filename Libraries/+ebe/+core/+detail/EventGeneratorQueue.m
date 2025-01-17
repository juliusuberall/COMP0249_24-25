classdef EventGeneratorQueue < handle

    properties(Access = protected)
        times;
        generators;
    end

    methods(Access = public)

        function clear(obj)
            obj.times = [];
            obj.generators = {};
        end

        function e = empty(obj)
            e = isempty(obj.times);
        end

        function insert(obj, time, eventGenerator)

            obj.times(end+1) = time;
            obj.generators{end+1} = eventGenerator;
            assert(length(obj.times) == length(obj.generators));

        end

        function insertWithDither(obj, time, dither, eventGenerator)

            obj.times(end+1) = time + rand * dither;
            obj.generators{end+1} = eventGenerator;
            assert(length(obj.times) == length(obj.generators));

        end

        function [time, eventGenerator] = pop(obj)

            % Handle case of no pending events
            if (isempty(obj.times) == true)
                time = [];
                eventGenerator = [];
                return
            end

            % Find the earliest time
            [time, idx] = min(obj.times);
            eg = obj.generators(idx);
            eventGenerator = eg{1};
            
            % Remove the items from the queue
            obj.times(idx) = [];
            obj.generators(idx) = [];
        end
    end
end