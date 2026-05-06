function toggleComment(filename, targetLineStart, action)
    % filename: name of the text file
    % targetLineStart: the start of the target line, e.g., 'screw1'
    % action: 'add' to add a # or 'remove' to remove a #
    
    % Read the file contents
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    fileContents = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = fileContents{1};
    
    % Process each line
    for i = 1:length(lines)
        line = lines{i};
        if contains(strtrim(line), targetLineStart) % Check if the targetLineStart is in the line
            if strcmp(action, 'add') && ~startsWith(strtrim(line), '#')
                lines{i} = ['#' line]; % Add the # at the beginning
            elseif strcmp(action, 'remove') && startsWith(strtrim(line), '#')
                lines{i} = strtrim(line(2:end)); % Remove the # at the beginning and trim spaces
            end
        end
    end
    
    % Write the modified contents back to the file
    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    for i = 1:length(lines)
        fprintf(fid, '%s\n', lines{i});
    end
    fclose(fid);
    
    %disp('File updated successfully.');
end
