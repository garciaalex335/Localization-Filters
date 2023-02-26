function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
     
    res = zeros(8,length(id));

    dimTag = 0.152; %m
    dimSpace = 0.152; %m

    dim34_67 = 0.178;

    % determine zero indexed row and column
    for i=1:length(id)
        row(i) = mod(id(i),12);
        column(i) = mod(floor(id(i)/12),9);
    end

    res(1,:) = (row(:)).*(2*dimTag) + dimTag; % lower left corners X
    res(3,:) = res(1,:); % lower right corners X
    res(5,:) = res(1,:)- dimTag; % upper right corners X
    res(7,:) = res(5,:); % upper left corners X



    %upper left corner Y
    for j = 1:length(id)
        if column(j) > 5 % need to offset y twice
            res(8,j) = (column(j)).*(2*dimTag) - (2*dimSpace) + (2*dim34_67); 
        elseif column(j) > 2 % need to offset y once
            res(8,j) = (column(j)).*(2*dimTag) - (dimSpace) + (dim34_67);
        else % no y offset
            res(8,j) = (column(j)).*(2*dimTag);
        end
    end

    res(6,:) = res(8,:) + dimTag; % upper right corners Y
    res(4,:) = res(6,:); % lower right corners Y
    res(2,:) = res(8,:); % lower left corners Y


    %Reorder rows to match with order from data
  %  res = [res(7,:);res(8,:);res(5,:);res(6,:);res(3,:);res(4,:);res(1,:);res(2,:)]; 

    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end