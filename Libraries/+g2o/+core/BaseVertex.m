% A base vertex stores a state value to be estimated. This could include
% the pose (position and orientation of a robot), a landmark, or almost any
% other type and kind of quantity of interest. The only requirement is that
% the state can be described using a state vector.
%
% The class constructor is hidden. To create a vertex, you need to create a
% subclass which specifies the dimension of the state. If the state vector
% sits on a nonlinear manifold (e.g., it's SE(2)), then you should
% overwrite the oplus method.
%
% Vertices can be "fixed". This means that we condition on the value - it
% is assumed to be known and fixed.

classdef BaseVertex < g2o.core.HyperGraphElement

    properties(Access = protected)
        
        % The dimension of the state estimate
        dimX;
        
    end
    
    properties(Access = {?g2o.core.HyperGraphElement,?g2o.core.OptimizableGraph})
        % The indices in the big X array used in the optimizable graph
        % We let the optimizable graph access it directly to speed things
        % up a bit.
        iX;    
        
        % Specify if a vertex is fixed
        conditioned;
        
        % The state estimate
        x;
                        
        % The edges connected to this vertex
        edgesMap;
        
        % Cached for speed
        edgesArray;
        updateEdgesArray;
    end
    
    methods(Access = protected)
       
        function obj = BaseVertex(dimension)
            obj = obj@g2o.core.HyperGraphElement();
            
            assert(nargin > 0, 'g2o:basevertex:basevertex:insufficientarguments', ...
                'The dimension is mandatory');
            
            % Check the dimensions are okay and store
            assert(dimension > 0, 'g2o:basevertex::basevertex:dimensionwrong', ...
                'The dimension must be a non-negative integer; the value is %d', dimension);
            
            obj.dimX = dimension;
            
            % Allocate an initial value
            obj.x = NaN(obj.dimX, 1);
            
            % Not fixed by default
            obj.conditioned = false;
            
            % Automatically assign the Id
            obj.setId(g2o.core.BaseVertex.allocateId());
            
            % Preallocate
            obj.edgesMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            obj.updateEdgesArray = true;
        end
    end
    
    methods(Access = public, Sealed = true)
        
        % Dimension of the state
        function dimension = dimension(obj)
            dimension = obj.dimX;
        end
        
        % Get the list of vertices associated with this vertex
        function edges = edges(obj)
            
            if (obj.updateEdgesArray == true)
                obj.edgesArray = values(obj.edgesMap);
                obj.updateEdgesArray = false;
            end
            edges = obj.edgesArray;
        end
        
        % Get the number of edges this vertex is attached to.
        function numEdges = numberOfEdges(obj)
            numEdges = length(obj.edgesMap);
        end
        
        % Return the current value of the estimate stored in the vertex.
        function estimate = estimate(obj)
            estimate = obj.x;
        end
        
        % Specify if the vertex state is condition. This means we assume
        % it's value is given / perfectly known.
        function conditioned = fixed(obj)
            conditioned = obj.conditioned;
        end
        
        % Set whether the vertex is fixed or not.
        function setFixed(obj, conditioned)
            obj.conditioned = conditioned;
        end
        
        % Obtain the hessian index for the vertex states. Internally, the
        % optimiser builds a single giant state vector. The values here are
        % the indices which map the state here to that vector.
        %
        % If a vertex if fixed, it is not included in the state vector and
        % the index will be empty.
        
        function xIndices = hessianIndex(obj)
            xIndices = obj.iX;
        end
        
        % Set the estimate. This needs to be a column vector of the right
        % dimension
        function setEstimate(obj, newX)
            
            % Needs to be a 2D array
            newXNDims = ndims(newX);
            assert(newXNDims == 2, ...
                'g2o:basevertex:setestimate:estimatewrongdimension', ...
                'The estimate vector must be a column vector; ndims=%d', ...
                newXNDims);
            
            % Get the vector sizes
            rows = size(newX, 1);
            cols = size(newX, 2);

            % Check number of rows
            assert(cols == 1, ...
                'g2o:basevertex:setestimate:estimatewrongdimension', ...
                'The estimate vector must be a column vector; columns=%d', ...
                cols);
            
            % Check number of columns
            assert(rows == obj.dimension, ...
                'g2o:basevertex:setestimate:estimatewrongdimension', ...
                'The estimate dimension is wrong; required=%d, actual=%d', ...
                obj.dimX, rows);
            
            % Check not NaN
            assert(any(isnan(newX)) == false, ...
                'g2o:basevertex:setestimate:estimatehasnans', ...
                'The estimate contains NaNs');

            obj.x = newX;
        end
        
        % The checks above have ensured the state dimension is correct;
        % this just checks for NaNs
        function validate(obj)
            
            % If already validated, return
            if (obj.validated == true)
                return
            end
            
            % Check if any NaNs are there
            % Check the dimension; this should be superfluous
            assert(any(isnan(obj.x)) == false, ...
                'g2o:basevertex:validate:estimatehasnans', ...
                'The estimate contains NaNs');
            
            % Check all the edges and make sure they are registered
            edges = values(obj.edgesMap);
            for e = 1 : length(edges)
                assert(edges{e}.owningGraph == obj.owningGraph, ...
                    'g2o:basevertex:validate:edgenotregistered', ...
                    'edge with ID % is not registered with a graph', obj.edges{e}.elementId);
            end            
            obj.validated = true;
        end
        
    end
    
    methods(Access = public)
        % This method handles the case that we add an update to the
        % estimate. The update comes from one step in the optimizer and
        % typically the value will be fairly small. This can be over-ridden
        % because the state might be on a nonlinear manifold. The most common
        % examples are angles (because of discontinuities) and quaternions.
        function oplus(obj, update)
            obj.x = obj.x + update;
        end
        
        % Set the estimate to its "zero state". This is included because
        % the "zero state" doesn't always mean the state vector is zero.
        % The most common example is if the state is a normalized
        % quaternion in which case zero is [0,0,0,1].
        function setToOrigin(obj)
            obj.x = zeros(1, obj.dimX);
        end
    end
    
    methods(Access = {?g2o.core.HyperGraph,?g2o.core.BaseEdge}, Sealed = true)
        
        % Helper function to add an edge.
        function obj = addEdge(obj, edge)
            obj.edgesMap(edge.id) = edge;
            obj.updateEdgesArray = true;
        end
        
        % Helper function to remove an edge.
        function obj = removeEdge(obj, edge)
            assert(isKey(obj.edgesMap, edge.id) == true, 'g2o:basevertex:removeedge:repeatid', ...
                'Attempt to remove unregistered edge %d', edge.id);
            remove(obj.edgesMap, edge.id);
            obj.updateEdgesArray = true;
        end
    end
    
    methods(Access = {?g2o.core.OptimizableGraph}, Sealed = true)
        
        % Internal function to set the indices.
        function setXIndices(obj, xIndices)
            obj.iX = xIndices;
        end        
    end
    
           
     methods(Access = protected, Static)
       
         % This private method ensures that each vertex has a unique ID.
         % This is used for map storage / search internally.
        function id = allocateId()
            persistent idCount;
            
            if (isempty(idCount))
                idCount = 0;
            else
                idCount = idCount + 1;
                
            end
            id = idCount;
        end
     end
end