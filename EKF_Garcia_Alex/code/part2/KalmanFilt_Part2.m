clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(7:9,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
  
      %calculate dt
      dt = sampledTime(i)-prevTime;
      prevTime = sampledTime(i);

      %extract angular velocity
      omegaBody = sampledData(i).omg;

      %extract acceleration
      accelBody = sampledData(i).acc;

      % pass to prediction to return uBar and covarBar
     [covarBar,uBar] = pred_step(uPrev,covarPrev,omegaBody,accelBody,dt);

    %pass covBar, uBar, measurement to update step
    [uCurrent, covar_current] = upd_step(Z(:,i),covarBar, uBar);

    %write to saved states for plotting
    savedStates(:,i) = uCurrent;

    %update variables for the next step
    uPrev = uCurrent;
    covarPrev = covar_current;


end



plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);