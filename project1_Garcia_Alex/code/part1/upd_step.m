function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively



C = [eye(6),zeros(6,9)];

R = 500;
W = eye(6);


Kt = (covarEst*C')/(C*covarEst*C' + W*R*W');

covar_curr = covarEst - (Kt * C * covarEst);

gmu = C * uEst;

uCurr = uEst + Kt*(z_t - gmu);


end