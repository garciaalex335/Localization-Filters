function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e,n)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    pSuccess = 0.99;
    

    maxIterations = log(1-pSuccess)/log(1-e^4);
    threshold = 3;

    mostInliners = 3;

    bestModel = [0];

    Vel = zeros(6,1);

    
    for i=1:ceil(maxIterations)

        % determine random values to check
        ind = randperm(length(optPos));

        H = zeros(1,6);

        % create H from the 3 random values
        for ii=1:3
            p = [optPos(ind(ii),1);optPos(ind(ii),2);1];
            skewP = [0     -1   p(2);
                     1      0  -p(1);
                    -p(2)  p(1)   0];
    
            A = ( p  * [0 0 1])- eye(3);
            B = (eye(3) - p*[0 0 1]) * skewP;
        
    
            H = [H;A(1:2,:)./Z B(1:2,:)];
        end

        H = H(2:end,:);

        HtH = H'*H;
        HCross = HtH\H';

        checkOpt = optV(2*(ind(1)-1)+1:2*(ind(1)-1)+2);

        for jj=2:3
            checkOpt = [checkOpt;optV(2*(ind(jj)-1)+1:2*(ind(jj)-1)+2)];
        end
        
%         checkOpt = [checkOpt;optV(2*(ind(3)-1)+1:2*(ind(3)-1)+2)];
        vCheck = HCross*checkOpt;
        inliers = [0];

        for j=1:length(ind)-3
            extractOpt = optV(2*(j-1)+1:2*(j-1)+6);
%             extractOpt = [extractOpt;optV(2*(j-1)+3:2*(j-1)+4)];
%             extractOpt = [extractOpt;optV(2*(j-1)+5:2*(j-1)+6)];
            v = HCross*extractOpt;
            error = abs(v-vCheck);
            if error < threshold
                inliers = [inliers;j];
            
            end

        end
        inliers = inliers(2:end);

        if length(inliers) > mostInliners
             mostInliers = length(inliers);
             bestModel = inliers;
%              thisError = error/length(inliers);
% 
%             if thisError < bestErr
%                 bestFit = betterModel;
%                 bestError = thisError;
%             end


        end

    end



            H = zeros(1,6);
           flow = [0];

        % create H from the 3 random values
        if bestModel ~= 0
            for ii=1:length(bestModel)
    
                p = [optPos(bestModel(ii),1);optPos(bestModel(ii),2);1];
                skewP = [0     -1   p(2);
                         1      0  -p(1);
                        -p(2)  p(1)   0];
        
                A = ( p  * [0 0 1])- eye(3);
                B = (eye(3) - p*[0 0 1]) * skewP;
            
        
                H = [H;A(1:2,:)./Z B(1:2,:)];
    
               flow= [flow;optV(2*(bestModel(ii)-1)+1:2*(bestModel(ii)-1)+2)];
            end

            H = H(2:end,:);
            flow = flow(2:end);
    
            HtH = H'*H;
            HCross = HtH\H';
            Vel = HCross*flow;
        end



%         checkOpt = optV(2*(ind(1)-1)+1:2*(ind(1)-1)+2);
% 
%         for jj=2:3
%             checkOpt = [checkOpt;optV(2*(ind(jj)-1)+1:2*(ind(jj)-1)+2)];
%         end
        
%         checkOpt = [checkOpt;optV(2*(ind(3)-1)+1:2*(ind(3)-1)+2)];
       
% 
%     Vel = bestFit;



    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end