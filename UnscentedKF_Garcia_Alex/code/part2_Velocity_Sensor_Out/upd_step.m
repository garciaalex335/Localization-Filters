function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    

    %% setup UKF constants
    n = 15;    
    alpha = 0.001;
    K = 1;
    beta = 2;
    

    lambda = (alpha^2)*(n+K) - n;

    wM0 = lambda / (n + lambda);
    wC0 = (lambda / (n+lambda)) + (1-(alpha^2) + beta);
    w = 1 / (2*(n+lambda));


    V = 0.2;
    V = V*eye(3);

%% calculate sigma points
    P = sqrtm(covarEst);


    X0 = uEst;
    X = X0;
    for i=1:n
        Xi = uEst+sqrt(n+lambda)*P(:,i);
        X = [X,Xi];
    
    end
    
    for i=n+1:2*n
        Xi = uEst-sqrt(n+lambda)*P(:,i-n);
        X = [X,Xi];
    
    end

    %% extract velocities from camera
    V_Linear_camera_WRT_world_cameraFrame = z_t(1:3);
    Omega_camera_WRT_world_cameraFrame = z_t(4:6);

    
    Zt = zeros(3,2*n + 1);
    %% pass omega points through function to convert predicted velocity to the camera frame
    for j = 1:2*n+1
        V_Linear_cam = cam_to_rob(X(7:9,j),Omega_camera_WRT_world_cameraFrame,X(4:6,j));
        Zt(:,j) = V_Linear_cam;
    end

    %% extract estimated velocity and covar via summations

    zMuT = wM0*Zt(:,1);

    for k = 2:2*n+1
        zMuT = zMuT + w*Zt(:,k);
    end

    
    Ct = wC0*(X(1:15,1) - uEst)*(Zt(:,1)-zMuT)';
    
    for o=2:2*n+1
        Ct =Ct + (w*(X(1:15,o) - uEst)*(Zt(:,o)-zMuT)');
    end
    

    St = wC0*(Zt(:,1)-zMuT)*(Zt(:,1)-zMuT)';
    
    for o=2:2*n+1
        St =St + (w*(Zt(:,o)-zMuT)*(Zt(:,o)-zMuT)');  
    end
    
    %% add noise and calculate mu and covar
    St = St + V;

    
    Kt = Ct/St;
    
    covar_curr = covarEst - (Kt * St * Kt');
    
    
    uCurr = uEst + Kt*(V_Linear_camera_WRT_world_cameraFrame - zMuT);


end


function V_CWC = cam_to_rob(v_Body_in_World,omega_Cam,eulerAngs)

     R_B2W = calcR(eulerAngs);
     R_W2B = R_B2W';

               yaw = -pi/4;

     Rot_CamToIMU_Z = [cos(yaw) -sin(yaw) 0; 
         sin(yaw) cos(yaw) 0;
         0 0 1];


     Rot_CamToIMU_X =[1 0 0;
                     0 -1 0;
                     0 0 -1];

          R_C2B = Rot_CamToIMU_Z*Rot_CamToIMU_X;

          R_B2C = R_C2B';



              trans_CamToRob = [-0.04;0;-0.03];

              trans_RobToCam = -R_C2B*trans_CamToRob;

              S = [0 -trans_RobToCam(3) trans_RobToCam(2);
                  trans_RobToCam(3) 0 -trans_RobToCam(1);
                  -trans_RobToCam(2) trans_RobToCam(1) 0];

              V_CWC = R_B2C*R_W2B*v_Body_in_World - R_B2C*S*omega_Cam;




end

function Rot = calcR(eulerAng)
phi = eulerAng(1);
theta = eulerAng(2);
psi = eulerAng(3);




%Initialize rotations
RZ = zeros(3,3);
RY = zeros(3,3);
RX = zeros(3,3);

%%%%%%%%%    RX
RX(1,1) = 1;
RX(2,1) = 0;
RX(3,1) = 0;

RX(1,2) = 0;
RX(2,2) = cos(phi);
RX(3,2) = sin(phi);

RX(1,3) = 0;
RX(2,3) = -sin(phi);
RX(3,3) = cos(phi);



RY(1,1) = cos(theta);
RY(2,1) = 0;
RY(3,1) = -sin(theta);

RY(1,2) = 0;
RY(2,2) = 1;
RY(3,2) = 0;

RY(1,3) = sin(theta);
RY(2,3) = 0;
RY(3,3) = cos(theta);



%%%%%%% RZ
RZ(1,1) = cos(psi);
RZ(2,1) = sin(psi);
RZ(3,1) = 0;

RZ(1,2) = -sin(psi);
RZ(2,2) = cos(psi);
RZ(3,2) = 0;

RZ(1,3) = 0;
RZ(2,3) = 0;
RZ(3,3) = 1;


Rot = RZ*RY*RX;


end