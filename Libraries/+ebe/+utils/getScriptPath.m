function scriptPath = getScriptPath(levelsUp)
    % Default levelsUp to 1 (the immediate caller) if not provided
    if nargin < 1
        levelsUp = 1;
    end
    
    % Use dbstack to get the call stack
    stack = dbstack('-completenames');
    
    % Ensure levelsUp is within the stack range; if not, get the pwd
    if numel(stack) > levelsUp
        % Get the file of the desired level in the stack
        callingFile = stack(levelsUp + 1).file; % +1 because stack(1) is getScriptPath itself
        scriptPath = fileparts(callingFile); % Extract the path
    else
        scriptPath = pwd;
    end
end
