% This class defines an action which gets fired after the graphics have
% been drawn

classdef PostDrawAction < ebe.core.Component
   
    methods(Access = public, Abstract = true)
        
        run(obj);
        
    end
    
end