function prepareDirectory(filePath, cleanDirectoryIfItExists)

    if (nargin == 1)
        cleanDirectoryIfItExists = true;
    end

    % First convert fileName to have the correct file separation for the
    % platform type
    filePath = strrep(filePath, '/', filesep);

    % Extract the directory part of the filepath
    [dirPath, ~, ~] = fileparts(filePath);

    % Check if the directory part is non-empty
    if ~isempty(dirPath)
        % Check if the directory exists
        if ((isfolder(dirPath) == true) && (cleanDirectoryIfItExists == true))
            % If the directory exists, delete its contents
            files = dir(fullfile(dirPath, '*')); % List all files and folders in the directory
            for k = 1:length(files)
                if ~files(k).isdir
                    % Delete file
                    delete(fullfile(dirPath, files(k).name));
                elseif ~ismember(files(k).name, {'.', '..'})
                    % Delete subdirectory and its contents
                    rmdir(fullfile(dirPath, files(k).name), 's');
                end
            end
            fprintf('Contents of the directory "%s" have been deleted.\n', dirPath);
        else
            % If the directory doesn't exist, create it
            mkdir(dirPath);
            fprintf('Created directories: %s\n', dirPath);
        end
    else
        fprintf('No directory specified in the filepath.\n');
    end
end
