clear
global M T dt

M = 100;
T = 6;
dt = 0.1;

getOrigTrajectory = false;

%noiseDist = makedist("Normal");    %%%%%%%%%%%%%%%%%%%%%%%5 change this to get specific variance
%noise_ts = random(noiseDist, [T/dt, 5]);
%random("normal", T/dt, 5, -1, 1, %rand(T*M, 5) - 0.5; % matrix of noises for T*M rows, 3 cols for x's and 2 for u's
% fix this noise to be a proper distribution%%%%%%%%%%%%%%%%%%%%%%%%%%%%



xprev = [0; 0; pi/2]; %%%%%%%%%%%%%%%%%%%%%%%%%%%% MAKE an x_t matrix with noise for init
uprev = [1 -0.5];

xprevMat = [];
uprevMat = [];

%xmats all

for ind=1:M
    xprevMat(:,ind) = xprev;
    uprevMat(:,ind) = uprev;
end

xmeans = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Perform timesteps
for index=1:T/dt
    
    % Start with belief at time t-1
    % Then predict belief using dynamics x_dot = u and some amount of noise
    % Then use measurement z_t to calculate weights for the particles
    % Then resample from the weighted distribution
    timeval = dt * index;
    
    x_tminus1 = xprevMat;
    u_tminus1 = uprevMat;
    x_t = [];
    x_bar_t = [];
    weights_t = [];
    %%%%%%%%%%%%%%%%%%%        ??????????? How to get the weight, how to
    %%%%%%%%%%%%%%%%%%%        "sample"
    %noise_t = noise_ts(index,:);
    
    % the normal distribution taken from stackoverflow and matlab posts on
    % normal distributions
%     v = 0.02; % variance
%     sigma = sqrt(v); % standard deviation
%     mu = 0; % mean
%     n = 1000
%     X = sigma .* randn(n, 1) + mu;
    
    noiseDist = makedist("Normal");    %%%%%%%%%%%%%%%%%%%%%%%5 change this to get specific variance
    
    %noise_ts_m = 0.02 * random(noiseDist, [M, 5]);
    noise_ts_m = random(noiseDist, [M, 5]);
    
    % PREDICTION    
    for m=1:M
        
        noisevariance = 0.1;
        noisesigmaval = sqrt(noisevariance);
        noisemeanval = 0;
        noise_t_w_v = noisesigmaval.*randn(5,1);% + b;
        %noise_t_w = noise_t_w_v(1,:);
        %noise_t_v = transpose(noise_t_w_v(2,:));
        
        %noise_t = noise_ts_m(m,:);
        noise_t = noise_t_w_v;
        
        % sample x_t_m from p(x_t|u_t, x_t-1_m)
        if getOrigTrajectory
            x_t_m_dot = xdotWithOneXAndNoise(x_tminus1(:,m), u_tminus1(:,m), [0;0;0]);%noise_t);
        else
            x_t_m_dot = xdotWithOneXAndNoise(x_tminus1(:,m), u_tminus1(:,m), noise_t);
        end
        x_t_m_sample = [x_tminus1(1) + dt * x_t_m_dot(1); x_tminus1(2) + dt * x_t_m_dot(2); x_tminus1(3) + dt * x_t_m_dot(3)];
        % set weights w_t_m = p(z_t|x_t_m)
        w_t_m = getProb(x_t_m_sample);
        weights_t(m) = w_t_m;
        % add pair (x_t_m, w_t_m) to X_bar_t
        x_bar_t(:,m) = [x_t_m_sample(1); x_t_m_sample(2); x_t_m_sample(3) ; w_t_m];
    end
    
    %weights_t;
    
    weightSumsOrig = sum(weights_t);
    tmp = x_bar_t;
    tmp(4,:) = tmp(4,:)/weightSumsOrig;
    x_bar_t_redone_weights = tmp;
    x_bar_t_redone_weights = transpose(sortrows(transpose(x_bar_t_redone_weights), 4));
    
    sums = [];
    sums(1) = x_bar_t_redone_weights(4,1);
    for m=2:M
        sums(m) = sums(m-1) + x_bar_t_redone_weights(4,m);
    end

    % RESAMPLING
    for m=1:M
        % draw i with probability w_t_m
        r = rand(); % get a rand in [0,1] uniform dist
        start = -1; % initialize val
        keepGoing = 1; % initial behavior is to keep checking
        ind = M; % default value is the last one with highest probability
        last = 1;
        for n=1:M
            start = sums(n);
            if (keepGoing == 1) & (start >= r)
                ind = last;%n;
                keepGoing = 0;
            end
            last = n;
        end
        resampled_x_t_i = x_bar_t_redone_weights(:,ind);
        % add x_t_i to X_t
        x_t(m,:) = resampled_x_t_i;
    end
    
    xprevMat = transpose(x_t(:,1:3));
    xmeans(:, index) = [mean(x_t(:,1)); mean(x_t(:,2))];
    
    %for k = 1:M
        %plot(timeval * ones(M, 1),)
    %    plot(x_t(k,1),x_t(k,2))
    %    if k == 1
    %       hold on
    %    end
    %end
    %hold off
    plot(x_t(:,1),x_t(:,2),"o")
    hold on
    
    
end
plot(xmeans(1,:),xmeans(2,:), "k-")
xlim([-1 4]); 
ylim([-1 5]);
title("Position");
xlabel("x1");
ylabel("x2");
hold off






























%%%%%%%%%%%%%%% do specific variance
%specific variance val and such






















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xvectdot = xdotWithFullXAndIndex(x, u, index)
  xvectdot = [cos(x(3, index)) * u(1, index); sin(x(3, index)) * u(1, index); u(2, index)];
end

function xvectdot = xdotWithOneX(x, u)
  xvectdot = [cos(x(3)) * u(1); sin(x(3)) * u(1); u(2)];
end

function xvectdot = xdotWithOneXAndNoise(x, u, noise)
  xvectdot = [cos(x(3)) * u(1) + noise(1); sin(x(3)) * u(1) + noise(2); u(2) + noise(3)];
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