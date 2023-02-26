%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

RUNRANSAC = 1; %MAKE A 1 TO RUN RANSAC, 0 FOR NO RANSAC

[sampledData, sampledVicon, sampledTime] = init(datasetNum);
tic
%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

    


     K = [311.0520 0 201.8724;
         0 311.3885 113.6210;
         0 0 1];

%% Rotations and Translation
          yaw = -pi/4;

     Rot_CamToIMU_X =[1 0 0;
                     0 -1 0;
                     0 0 -1];

    trans_CamToRob = [-0.04;0;-0.03];

     Rot_CamToIMU_Z = [cos(yaw) -sin(yaw) 0; 
         sin(yaw) cos(yaw) 0;
         0 0 1];

     R_c2r = Rot_CamToIMU_Z*Rot_CamToIMU_X;

     T_c2r = [R_c2r,trans_CamToRob;0 0 0 1];

     T_r2c = inv(T_c2r);



    e3 = [0;0;1];


    %% extracting dt and applying a low pass
    dt = [sampledData(2:end).t] - [sampledData(1:end-1).t];

    dt = lowpass(dt,0.01);

    estimatedV = zeros(6,length(sampledData));


for n = 2:length(sampledData)
    %% Initalize Loop load 
    
    
    imLast = sampledData(n-1).img;
    im = sampledData(n).img;


    %% Detect good points
    cornersLast = detectHarrisFeatures(imLast);
    
%     imshow(imLast);hold on;


    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker;
    initialize(pointTracker,cornersLast.Location,imLast);

    %% Find the location of the next points;
    [corners,validity] = pointTracker(im);
    
    % extract points that are valid 
    % (done by passing validity to the row indexes)
    validPrev = cornersLast.Location(:,:);
    validCurr = corners(:,:);

    [position, orientation, R_c2w] = estimatePose(sampledData(n), sampledData(n).t);


    Zc = position(3);


    XcPrev = (validPrev(:,1) - K(1,3)) /K(1,1);
    XcCurr = (validCurr(:,1) - K(1,3)) /K(1,1);    


    YcPrev = (validPrev(:,2) - K(2,3)) /K(2,2);
    YcCurr = (validCurr(:,2) - K(2,3)) /K(2,2);  

    %% Calculate velocity
    % Use a for loop


    u = (XcCurr(:) - XcPrev(:))/dt(n-1);
    v = (YcCurr(:) - YcPrev(:))/dt(n-1);
    
    

    H = zeros(1,6);
    uv = zeros(length(u)*2,1);

    for i=1:length(u)
        % build optical flow vector
        uv(2*(i-1) + 1,1) = u(i);
        uv(2*(i-1) + 2,1) = v(i);
%   

        % calculate H matrix
        if RUNRANSAC == 0
            skewP = [0 -1 YcCurr(i);
                1 0 -XcCurr(i);
                -YcCurr(i) XcCurr(i) 0];
    
            A = ( [XcCurr(i);YcCurr(i);1]  * e3')- eye(3);
            B = (eye(3) - [XcCurr(i);YcCurr(i);1] * e3') * skewP;
        
    
            H = [H;A(1:2,:)./Zc B(1:2,:)];
        end
    end
% 
    if RUNRANSAC == 0
        H = H(2:end,:);
    
        HtH = H'*H;
        HCross = HtH\H';
        V_CamRelWorld =HCross*uv;
    end
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    if RUNRANSAC == 1
        V_CamRelWorld = velocityRANSAC(uv,[XcCurr,YcCurr],Zc,R_c2w,0.8);
    end
    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    sk = [0 -trans_CamToRob(3) trans_CamToRob(2) ;
        trans_CamToRob(3) 0 -trans_CamToRob(1);
        -trans_CamToRob(2) trans_CamToRob(1) 0];
    
    adjoint = [R_c2r -R_c2r*sk; 
        zeros(3) R_c2r];

    Vel_RobRelWorld = adjoint*V_CamRelWorld;

    Vel = [eul2rotm(orientation),zeros(3);
        zeros(3) eul2rotm(orientation)]*Vel_RobRelWorld;
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 


    estimatedV(:,n) = Vel;
   
    if n == length(sampledData)
        estimatedV(1,:) = lowpass(estimatedV(1,:),0.01);
        estimatedV(2,:) = lowpass(estimatedV(2,:),0.01);
        estimatedV(3,:) = lowpass(estimatedV(3,:),0.01);

        estimatedV(4,:) = lowpass(estimatedV(4,:),0.05);
        estimatedV(5,:) = lowpass(estimatedV(5,:),0.05);
        estimatedV(6,:) = lowpass(estimatedV(6,:),0.05);
    end

    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    %estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
toc