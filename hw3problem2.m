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