% List of component paths
components = {
    'models/jawmodel/rigidBodies/jaw', ...
    'models/jawmodel/rigidBodies/jaw_resected', ...
    'models/jawmodel/rigidBodies/skull', ...

};

% Loop through each component and set visibility to false
for i = 1:length(components)
    % Find the component
    component = ah1.find(components{i});
    
    % Check if the component exists
    if ~isempty(component)
        % Get the RenderProps of the component
        renderProps = component.getRenderProps();
        
        % Set the visibility to false
        renderProps.setVisible(true);
    else
        % Warn if the component is not found
        warning('Component %s not found.', components{i});
    end
end

% List of component paths
components = {
    'models/jawmodel/rigidBodies/deformed_jaw', ...
    'models/jawmodel/rigidBodies/deformed_skull', ...
};

% Loop through each component and set visibility to false
for i = 1:length(components)
    % Find the component
    component = ah1.find(components{i});
    
    % Check if the component exists
    if ~isempty(component)
        % Get the RenderProps of the component
        renderProps = component.getRenderProps();
        
        % Set the visibility to false
        renderProps.setVisible(false);
    else
        % Warn if the component is not found
        warning('Component %s not found.', components{i});
    end
end



