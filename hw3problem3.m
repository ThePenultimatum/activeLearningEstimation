clear
global M T dt

M = 1;
T = 1;
dt = 0.1;

getOrigTrajectory = false;

Ak = [0 1; -1 0];
noisevariance = 0.1;
noisesigmaval = sqrt(noisevariance);
noisemeanval = 0;
P_kminus1_given_kminus1 = [1 0; 0 1];
xhat_kminus1_given_kminus1 = [1; 1];
C_k = [1 0; 0 1];
R_k = [1 0; 0 1];

xhats = [];
xpredictions = [];
pkkminus1s = [];

Ak = [0 1; -1 0];
perturbed_P_kminus1_given_kminus1 = [1 0; 0 1];
perturbed_xhat_kminus1_given_kminus1 = [1; 1];
C_k = [1 0; 0 1];
R_k = [1 0; 0 1];

perturbedXhats = [];
perturbedXpredictions = [];
perturbedKks = [];

xhatsAll = [];
xpredictionsAll = [];
pkkminus1sAll = [];
perturbedXhatsAll = [];
perturbedXpredictionsAll = [];
perturbedKksAll = [];

originalTrajectoryValsAll = [];

for m=1:M
    
    Ak = [0 1; -1 0];
    P_kminus1_given_kminus1 = [1 0; 0 1];
    xhat_kminus1_given_kminus1 = [1; 1];
    perturbed_P_kminus1_given_kminus1 = [1 0; 0 1];
    perturbed_xhat_kminus1_given_kminus1 = [1; 1];
    C_k = [1 0; 0 1];
    R_k = [1 0; 0 1];
    
    origxprev = [1; 1];
    originalTrajectoryVals_t = [origxprev];
    
    for i=1:(T/dt)
        
        % original trajectory
        neworig = origxprev + dt * xdotWithOneXAndNoise(origxprev, [0, 0]);
        originalTrajectoryVals_t(:,i+1) = origxprev + dt * xdotWithOneXAndNoise(origxprev, [0, 0]);
        origxprev = neworig;
        
        %%% Prediction phase
        % Calculate the noise to be used
        noise_t_w_v = noisesigmaval.*randn(2,2);% + b;
        noise_t_w = noise_t_w_v(1,:);
        noise_t_v = transpose(noise_t_w_v(2,:));
        % calculate the prediction of x with the noise
        xhat_k_given_kminus1 = xhat_kminus1_given_kminus1 + dt * xdotWithOneXAndNoise(xhat_kminus1_given_kminus1, noise_t_w);
        xpredictions(:,i) = xhat_k_given_kminus1;
        z_k = xhat_k_given_kminus1;% + noise_t_v; % would also potentially have a measurement model by which to multiply newx
        % Calculate the update to the prediction covariance
        P_k_given_kminus1 = Ak * P_kminus1_given_kminus1 * transpose(Ak); % + Q_k; % Q_k from noise, assume mean 0 so this can be skipped?
        pkkminus1s(:,:,i) = P_k_given_kminus1;
        %
        %%% Measurement update phase
        S_k = R_k + P_k_given_kminus1;
        % Calculate Kalman gain assuming using deriv of trace because easier
        % derivative
        K_k = P_k_given_kminus1 * transpose((C_k) * inv(C_k * P_k_given_kminus1 * transpose(C_k)) + R_k);
        % Calculate P_k_given_k which becomes the p_kminus1_given_kminus1 for
        % the next iteration
        %xprev = newx;
        xhat_k_given_k = xhat_k_given_kminus1 + K_k * (z_k - xhat_k_given_kminus1);
        meas_k_given_k = z_k - xhat_k_given_k;

        xhats(:,i) = xhat_k_given_k;

        P_kminus1_given_kminus1 = P_k_given_kminus1 - K_k * C_k * P_k_given_kminus1;
        xhat_kminus1_given_kminus1 = xhat_k_given_k;



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%% Commenting out the perturbation code to
        %%%%%%%%%%%%%%%%%%%%% change the way I am getting the perturbations
        %%%%%%%%%%%%%%%%%%%%% to just get the ideal one once and then
        %%%%%%%%%%%%%%%%%%%%% perturb that one 10 times and then
        %%%%%%%%%%%%%%%%%%%%% recalculate the results for the 10 different
        %%%%%%%%%%%%%%%%%%%%% series of K matrices

        %%% Perturbed prediction phase
        % Calculate the noise to be used
        %noise_t_w_v = noisesigmaval.*randn(2,2);% + b;
        %noise_t_w = noise_t_w_v(1,:);
        %noise_t_v = transpose(noise_t_w_v(2,:));
        % calculate the prediction of x with the noise
        perturbed_xhat_k_given_kminus1 = perturbed_xhat_kminus1_given_kminus1 + dt * xdotWithOneXAndNoise(perturbed_xhat_kminus1_given_kminus1, noise_t_w);
        perturbedXpredictions(:,i) = perturbed_xhat_k_given_kminus1;
        perturbed_z_k = perturbed_xhat_k_given_kminus1 + noise_t_v; % would also potentially have a measurement model by which to multiply newx
        % Calculate the update to the prediction covariance
        perturbed_P_k_given_kminus1 = Ak * perturbed_P_kminus1_given_kminus1 * transpose(Ak); % + Q_k; % Q_k from noise, assume mean 0 so this can be skipped?
        pkkminus1s(:,:,i) = perturbed_P_k_given_kminus1;
        %
        %%% Perturbed Measurement update phase
        perturbed_S_k = R_k + perturbed_P_k_given_kminus1;
        % Calculate Kalman gain assuming using deriv of trace because easier
        % derivative
        perturbed_K_k_optimal = perturbed_P_k_given_kminus1 * transpose((C_k) * inv(C_k * perturbed_P_k_given_kminus1 * transpose(C_k)) + R_k);
        % perturb kk
        noise_kk_perturb = sqrt(0.01).*randn(2,2);
        perturbed_K_k = perturbed_K_k_optimal + noise_kk_perturb;
        perturbedKks(:,:,i) = perturbed_K_k;
        % Calculate P_k_given_k which becomes the p_kminus1_given_kminus1 for
        % the next iteration
        %xprev = newx;
        perturbed_xhat_k_given_k = perturbed_xhat_k_given_kminus1 + perturbed_K_k * (perturbed_z_k - perturbed_xhat_k_given_kminus1);
        perturbed_meas_k_given_k = z_k - xhat_k_given_k;

        perturbedXhats(:,i) = perturbed_xhat_k_given_k;

        perturbed_P_kminus1_given_kminus1 = perturbed_P_k_given_kminus1 - K_k * C_k * perturbed_P_k_given_kminus1;
        perturbed_xhat_kminus1_given_kminus1 = perturbed_xhat_k_given_k;
    end
    
    xhatsAll = [xhatsAll, xhats];
    xpredictionsAll = [xpredictionsAll, xpredictions];
    pkkminus1sAll = [pkkminus1sAll, pkkminus1s]; %%%%%%%%%%%%%%%% WRONG APPENDING
    perturbedXhatsAll = [perturbedXhatsAll, perturbedXhats];
    perturbedXpredictionsAll = [perturbedXpredictionsAll, perturbedXpredictions];
    perturbedKksAll = [perturbedKksAll, perturbedKks];%%%%%%%%%%%%%%%% WRONG APPENDING
    
    
    
    plot([1, xpredictions(1,:)],[1, xpredictions(2,:)],"b");%,".")
    hold on;
    %plot(perturbedXpredictionsAll(1,:),perturbedXpredictionsAll(2,:),"g");%,".")
    plot([1,perturbedXpredictions(1,:)],[1,perturbedXpredictions(2,:)],"g");%,".")
    hold on;
    
end

vals1 = [];
vals2 = [];
vals3 = [];
vals4 = [];

for i=1:T/dt
    vals1(i) = pkkminus1s(1,1,i);
    vals2(i) = pkkminus1s(1,2,i);
    vals3(i) = pkkminus1s(2,1,i);
    vals4(i) = pkkminus1s(2,2,i);
end

plot(originalTrajectoryVals_t(1,:),originalTrajectoryVals_t(2,:),"r");
xlim([-10 10]); 
ylim([-10 10]);
title("Position");
xlabel("x1");
ylabel("x2");
%hold on;
%plot(originalTrajectoryVals_t(1,:),originalTrajectoryVals_t(2,:));
%hold on;
%plot(perturbedXpredictionsAll(1,:),perturbedXpredictionsAll(2,:));%,".")
hold off;


% plot(1:T/dt ,vals1);
% hold on;
% plot(1:T/dt ,vals2);
% hold on;
% plot(1:T/dt ,vals3);
% hold on;
% plot(1:T/dt ,vals4);
% title("Prediction covariance evolution");
% xlabel("value of entry in prediction covariance matrix");
% ylabel("time steps");
% hold off


% plot(xpredictions(1,:),xpredictions(2,:));%,".")
% xlim([-10 10]); 
% ylim([-10 10]);
% title("Position");
% xlabel("x1");
% ylabel("x2");
% hold on;
% plot(perturbedXpredictions(1,:),perturbedXpredictions(2,:));%,".")
% hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xvectdot = xdotWithFullXAndIndex(x, index)
  xvectdot = [x(2, index); -x(1, index)];
end

function xvectdot = xdotWithOneX(x)
  xvectdot = [x(2); -x(1)];
end

function xvectdot = xdotWithOneXAndNoise(x, noise)
  xvectdot = [x(2) + noise(1); -x(1) + noise(2)];
end

function xk = getXk(ak, xkminus1, noise_k)
  xk = ak * xkminus1;% + noise_k;
end

function zk = getZk(ck, xkminus1, noise_k)
  zk = ck * xkminus1 + noise_k;
end

function prob = getProb(x)
  %sigSquared = 1; % variance
  %mu = 0; % mean
  %x
  sigVarianceMat = [1 0 0; 0 1 0; 0 0 1]; %[1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];
  muMeanVect = [0; 0; 0]; %[0; 0; 0; 0; 0];
  diffVect = x-muMeanVect;
  %sqrt(det(2*pi*sigVarianceMat)) 
  %transpose(diffVect)*inv(sigVarianceMat)*diffVect
  %exp(-0.5*transpose(diffVect)*inv(sigVarianceMat)*diffVect)
  prob = (1/(sqrt(det(2*pi*sigVarianceMat)))) * exp(-0.5*transpose(diffVect)*inv(sigVarianceMat)*diffVect);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 check pos and neg;;;; is a probability, should be in [0,1]
end