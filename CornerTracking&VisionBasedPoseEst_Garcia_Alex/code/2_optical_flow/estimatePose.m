function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    ids = data.id;
      
    tagsWorld = getCorner(ids); % each column returns as [LLX LLY LRX LRY URX URY ULX ULY]'
    
    H = findH(data,tagsWorld);

     K = [311.0520 0 201.8724;
         0 311.3885 113.6210;
         0 0 1];
     
     RT = K\H;

     R1 = RT(:,1);
     R2 = RT(:,2);
     T_hat = RT(:,3);

     R3 = cross(R1,R2);

     [U,S,V] = svd([R1,R2,R3]);
    

     diag = [1 0 0; 
         0 1 0;
         0 0 det(U*V')];

     R = U*diag*V';
     T = T_hat/sqrt(R1(1)^2 + R1(2)^2 + R1(3)^2);

     WorldToCamera = [R;0 0 0];
     WorldToCamera = [WorldToCamera,[T;1]];
    
     CamToWorld = inv(WorldToCamera);
     R_c2w = CamToWorld(1:3,1:3);
     yaw = -pi/4;

     Rot_CamToIMU_X =[1 0 0;
                     0 -1 0;
                     0 0 -1];



     Rot_CamToIMU_Z = [cos(yaw) -sin(yaw) 0; 
         sin(yaw) cos(yaw) 0;
         0 0 1];

     Rot_CamToRob = Rot_CamToIMU_Z*Rot_CamToIMU_X;

    trans_CamToRob = [-0.04;0;-0.03];

     T_CamToRob = [Rot_CamToRob; 0 0 0];
     T_CamToRob = [T_CamToRob, [trans_CamToRob;1]];
     T_RobToCam = T_CamToRob;
  %   T_WorldToRob =WorldToCamera* T_CamToRob ;
%      T_RobToWorld = [T_WorldToRob(1:3,1:3)]';
%      T_RobToWorld = [T_RobToWorld;0 0 0];
%      T_RobToWorld = [T_RobToWorld,[ -[T_WorldToRob(1:3,1:3)]' * T_WorldToRob(1:3,4);1]];
%     %T_RobToWorld = inv(T_WorldToRob);
    T_RobToWorld = CamToWorld*T_RobToCam;

    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    position = T_RobToWorld(1:3,4);
    orientation = rotm2eul(T_RobToWorld(1:3,1:3));
end



function [H] = findH(data,worldCoor)


    x = zeros(9,1);
    
    for i = 1:length(data.p1)
        
          x2 = [data.p1(:,i);1];
          x2 = [x2;[data.p2(:,i);1]];
          x2 = [x2;[data.p3(:,i);1]];
          x2 = [x2;[data.p4(:,i);1]];
    
    
     skewX2_1 = [0 -1 x2(2);
               1 0 -(x2(1));
         -(x2(2)) x2(1) 0];
    
      skewX2_2 = [0 -1 x2(5);
                1 0 -(x2(4));
         -(x2(5)) x2(4) 0];
    
       skewX2_3 = [0 -1 x2(8);
         1 0 -(x2(7));
         -(x2(8)) x2(7) 0];
    
        skewX2_4 = [0 -1 x2(11);
         1 0 -(x2(10));
         -(x2(11)) x2(10) 0];


        x = [x,kron([worldCoor(1:2,i);1],skewX2_1)];
        x = [x,kron([worldCoor(3:4,i);1],skewX2_2)];
        x = [x,kron([worldCoor(5:6,i);1],skewX2_3)];
        x = [x,kron([worldCoor(7:8,i);1],skewX2_4)];

    end
    
    x = x(:,2:end);
    x = x';

    [Ux,Sx,Vx] = svd(x);

   % HL = Vx(:,9)/Vx(9,9)

    HL = [Vx(1:3,9),Vx(4:6,9),Vx(7:9,9)];


    Sh = svd(HL);

    H = HL/Sh(2);

    
    if [x2(1:2);1]'*H*[worldCoor(1:2,end);1] < 0
        H = -H;
    end

    
    


end