function jsonFileContent = readJSONFile(fileName, levelUp)

if (nargin == 1)
    levelUp = 0;
end

jsonFileName = ebe.utils.resolveFilePath(fileName, 3 + levelUp);
jsonFile = fileread(jsonFileName);

try
    jsonFileContent = jsondecode(jsonFile);
catch ME
    error('Failed to read or decode JSON file: %s\n%s', jsonFile, ME.message);
end

% Now loop through and find every field with the value *.json; assume this
% is a configuration file, load it and replace the field with the file
% content
%
% WARNING: This could get stuck in an infinite loop

% Get all field names in the struct
fields = fieldnames(jsonFileContent);
    
% Copy the original struct to modify it
updatedJSONFileContent = jsonFileContent;
    
% Loop through each field
for i = 1:numel(fields)
    % Get the value of the current field
    fieldValue = jsonFileContent.(fields{i});
    
    % Check if the value is a string or char array and matches '*.json'
    if (ischar(fieldValue) || isstring(fieldValue)) && endsWith(fieldValue, '.json')
        % Try to read and decode the JSON file
        try
            jsonContent = ebe.utils.readJSONFile(fieldValue, levelUp + 1);
            updatedJSONFileContent.(fields{i}) = jsonContent;
            fprintf('Replaced field "%s" with JSON-decoded content from %s\n', fields{i}, fieldValue);
        catch ME
            error('Failed to read or decode JSON file: %s\n%s', fieldValue, ME.message);
        end
    end
end

jsonFileContent = updatedJSONFileContent;