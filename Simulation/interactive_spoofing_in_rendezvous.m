% Venkatraman Renganathan
% W_MSR Code
% Input initial conditions of node values
% See the consensus converging despite having F malicious nodes

addpath('Images')

% clear all; 
close all; clc;

%% Setting parameters

% A = [0 1 1 0 0 1 
%      1 0 1 1 0 1
%      1 1 0 1 1 0
%      0 1 1 0 1 1 
%      0 0 1 1 0 1
%      1 1 0 1 1 0];              % Adjacency matrix
% A = [0 1 1
%      1 0 1 
%      1 1 0];              % Adjacency matrix
A = [0 1 1 1
     1 0 1 1 
     1 1 0 1
     1 1 1 0];              % Adjacency matrix
 
N = size(A,1);                   % Number of nodes
F = 1;                           % Number of malicious nodes
idxMal = [3];                    % Index of the malicious nodes
idxSpf = [];                     % Index of the spoofed agents

time_span = 50;                  % Simulation time

 
% Random initil positions  
a = 0;
b = 50;
x0 = (b-a).*rand(N,1) + a;
y0 = (b-a).*rand(N,1) + a;

% Initial positions of the malicious and spoofed nodes
x0(idxMal) = 20;
y0(idxMal) = 20;
x0(idxSpf) = 15;
y0(idxSpf) = 15;

% N-agent polygon formation in n-gon
qAng = linspace(0,360,N+1); % Desired locations of agents on the unit circle given in angle
qAng(end) = [];
qf = 10.*[cosd(qAng); sind(qAng)]; % desired locations in x-y coordinates

qfx= qf(1,:).';
qfy= qf(2,:).';


%% Preallocate variables

par.N = N;              % Number of nodes
par.F = F;              % Number of malicious nodes
par.D = diag(sum(A,2)); % Degree matrix
par.A = A;              % Adjacency matrix
par.idxMal = idxMal;    % Index of malicious agents
par.idxSpf = idxSpf;    % Index of spoofed nodes

% Preallocate state vector
x = zeros(N, time_span+1);
y = zeros(N, time_span+1);
x(:,1) = x0;
y(:,1) = y0;

% x-y positions of robots
xRob = x;
yRob = y;

%% Simulation

for times = 1 : time_span
    % Update state of legitimate nodes
    x(:,times+1) = spoofing_wmsr_Ver1_2(x(:,times),par);
    y(:,times+1) = spoofing_wmsr_Ver1_2(y(:,times),par);
    
    % Bias vector
    xRob(:,times+1) = x(:,times+1) + qfx;
    yRob(:,times+1) = y(:,times+1) + qfy;
    
    % Update state of malicious and spoofed nodes
    x(idxMal,times+1) = x(idxMal,times) + 1; 
    y(idxMal,times+1) = y(idxMal,times);
    x(idxSpf,times+1) = x(idxSpf,times) + 1; 
    y(idxSpf,times+1) = y(idxSpf,times);    
end



%% Plot

% plot(x.',y.')
% figure; plot(x.'); title('x pos vs. time'); xlabel('t')
% figure; plot(y.'); title('y pos vs. time'); xlabel('t')

plotParam.adj = A;     
plotParam.N = N;
plotParam.x = xRob';
plotParam.y = yRob';
plotParam.theta = zeros(size(x.'));
MoviePlane(plotParam)