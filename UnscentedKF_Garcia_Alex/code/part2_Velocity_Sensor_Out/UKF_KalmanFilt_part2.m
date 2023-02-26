clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;

%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP

      %calculate dt
      dt = sampledTime(i)-prevTime;
      prevTime = sampledTime(i);

      %extract angular velocity
      omegaBody = sampledData(i).omg;

      %extract acceleration
      accelBody = sampledData(i).acc;

      [covarBar,uBar] = pred_step(uPrev,covarPrev,omegaBody,accelBody,dt);



    zt = [vel(i,:)';angVel2(i,:)'];

    %pass covBar, uBar, measurement to update step
    [uCurrent, covar_current] = upd_step(zt,covarBar, uBar);

    %write to saved states for plotting
    savedStates(:,i) = uCurrent;

    %update variables for the next step
    uPrev = uCurrent;
    covarPrev = covar_current;

end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);