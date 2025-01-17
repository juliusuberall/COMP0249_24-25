% This class stores events in an ordered queue. Sorting is based on
% increasing time value.

classdef OrderedEventQueue < handle

    properties(Access = protected)
        eventQueue;
        dirty;
    end
        
    methods(Access = public)
        
        function obj = OrderedEventQueue()
            obj.clear();
        end
        
        function clear(obj)
            obj.eventQueue = {};
            obj.dirty = false;
        end
        
        function insert(obj, newEvents)

            % Flag as dirty
            obj.dirty = true;
            
            % Handle the case of a single event first
            if (iscell(newEvents) == false)
                obj.eventQueue{end+1} = newEvents;
                return
            end
            
            % Now handle the case of a cell array
            for c = 1 : length(newEvents)
                obj.eventQueue{end+1} = newEvents{c};
            end
        end
        
        function sortedEvents = events(obj)
            if (obj.dirty == true)
                times = cellfun(@(e) e.time, obj.eventQueue);
                [~, idx] = sort(times, 'ascend');
                obj.eventQueue = obj.eventQueue(idx);
                obj.dirty = false;
            end
            sortedEvents = obj.eventQueue;
        end        
    end
end
