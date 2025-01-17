function fullPath = resolveFilePath(fileName, levelsUp)

    if (nargin == 0)
        levelsUp = 2;
    end

    % First convert fileName to have the correct file separation for the
    % platform type
    fileName = strrep(fileName, '/', filesep);

    % Check if the given file path is absolute
    if isabsolute(fileName)
        % If it's absolute, just return it as-is
        fullPath = fileName;
    else
        % If it's relative, construct the absolute path using getScriptPath
        callingScriptPath = ebe.utils.getScriptPath(levelsUp); % Call your helper function
        fullPath = fullfile(callingScriptPath, fileName); % Combine paths
    end
end

% OS-independent way to figure out if this is an absolute path or not
function absFlag = isabsolute(path)
    % Check if the path is absolute
    if ispc % Windows
        absFlag = ~isempty(regexp(path, '^[a-zA-Z]:\\', 'once')); % Drive letter followed by '\'
    else % Mac/Linux
        absFlag = startsWith(path, '/'); % Absolute paths start with '/'
    end
end
