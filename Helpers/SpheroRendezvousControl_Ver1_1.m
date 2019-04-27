%%%%%%%%%%%%%%%%%%%
% Sphero  control %
%%%%%%%%%%%%%%%%%%%  
function SpheroState = SpheroRendezvousControl_Ver1_1(iitr,itr,j, SpheroState, SpheroStateLeader, CameraParam)


PosWorld    = SpheroState.PosWorld;     % Positions in world coordinate frame
PosKalm     = SpheroState.PosKalm;      % Position of robots from Kalman filter(in world frame)
ThtKalm     = SpheroState.ThtKalm;      % Estimated headings from Kalman filter
MotionIndex = SpheroState.MotionIndex;  % Index to identify motion
VelWorld    = SpheroState.VelWorld;     % Estimated speed
VelWorldFilt= SpheroState.VelWorldFilt; % Filtered estimated speed
Omega       = SpheroState.Omega;        % Angular velocity 
Time        = SpheroState.Time;         % CPU time at each iteration
Sph         = SpheroState.Sph;          % Spheros
VelCtrl     = SpheroState.VelCtrl;      % Speed from control vector
ThtCtrl     = SpheroState.ThtCtrl;      % Heading from control vector
VelInput    = SpheroState.VelInput;     % Input speed
VelSatInput = SpheroState.VelSatInput;  % Input speed
ThtInput    = SpheroState.ThtInput;     % Input heading
Theta0      = SpheroState.Theta0;       % Orientation bias

vMax        = SpheroState.Param.vMax;   % Maximum allowed velocity
Kp          = SpheroState.Param.Kp;     % P-Gain for PID control
Kd          = SpheroState.Param.Kd;     % D-Gain for PID control

dist        = SpheroState.Ctrl.dist;        % Distance between two robots
frameSize   = SpheroState.Ctrl.frameSize;   % Size of camer frame in pixels
thresh      = SpheroState.Ctrl.thresh;      % Distance in pixels from edges of the camera frame in which the Spheros are banned
incrmt      = SpheroState.Ctrl.incrmt;      % Increment angle in degrees (when searching for locations internal to the camera frame
Calib       = CameraParam.Calib;            % Interinsics of camera
R           = CameraParam.Rot;              % Rotation matrix of camera
T           = CameraParam.Tran;             % Translation vector of camera
PosPixel    = SpheroState.PosPixel;         % Pixel position of Spheros
PosPixelAll = SpheroState.PosPixelAll;      % Pixel postion of all detected objects
posDes      = SpheroState.Ctrl.posDes;      % Desired postion of the Spheros in the world frame
numRob      = SpheroState.numRob;           % number of robots

posMalicious  = SpheroStateLeader.PosPixelAll{iitr}; % Estimated position of leader


%% Control parameters

N = numRob + 1;            % Number of all nodes

% x-y position of the robots
xPos = PosWorld(1,:,iitr);
yPos = PosWorld(2,:,iitr);

F = 1;                      % Number of malicious nodes
idxMal = numRob + 1;        % Index of the malicious nodes
idxSpf = [];                % Index of the spoofed agents

A = ones(N,N) - diag(ones(N,1));  % Adjacency matrix

par.N = N;                  % Number of all nodes
par.F = F;                  % Number of malicious nodes
par.D = diag(sum(A,2));     % Degree matrix
par.A = A;                  % Adjacency matrix
par.idxMal = idxMal;        % Index of malicious agents
par.idxSpf = idxSpf;        % Index of spoofed nodes

radius = 250;               % Radius of the desired formation

% N-agent polygon formation (for legitimage robots)
qAng = linspace(0,360,numRob+1); % Desired locations of agents on the unit circle given in angle
qAng(end) = [];
qf = radius.*[cosd(qAng); sind(qAng)]; % Desired locations in x-y coordinates

% Position bias
qfx = qf(1,:);
qfy = qf(2,:);

% x-y positions fed to the algorithm
xAlg = xPos - qfx;
yAlg = yPos - qfy;


%% Spoof resilient control 

if any(isnan(posMalicious(:))) || size(posMalicious,2) ~= 1
    posMalx = 0;
    posMaly = 0;
    fprintf('No malicious agent detected.');
else
    % Reconstruct the 3D location of the malicous agents
    imPts = [posMalicious; 1];
    p     = Calib \ imPts;                         % Homogenious point coordinates on the image
    Nc    = R * [0; 0; 1];                         % Normal of the plane (given in the camara coordinate frame)
    d     = Nc.' * T;                              % Distance from camera to the plane
    Nxp   = Nc.' * p;
    P     = bsxfun(@rdivide, d*p, Nxp);            % Point coordinates (in camera coordinate frame)
    Pw    = bsxfun(@plus, R.'*P, -R.'*T);          % Point coordinates (in world coordinate frame)
    posMal      =  Pw(1:2,:);
    posMal(1,:) = -posMal(1,:);                  % Orient the world frame s.t. z-axis is pointing up

    posMalx = posMal(1);
    posMaly = posMal(2);
end  

% Control to bring the robots to the desired position

xAlgDes = [spoofing_wmsr_Ver1_2([xAlg posMalx].', par)].';
yAlgDes = [spoofing_wmsr_Ver1_2([yAlg posMaly].', par)].';

% Add bias
posDes(:,:,iitr) = [xAlgDes(1:numRob) + qfx;
                    yAlgDes(1:numRob) + qfy];


%% PID motion control 

ctrl = zeros(2,numRob);
for jj = 1 : numRob
    ctrl(:,jj) = posDes(:,jj,iitr) - PosWorld(:,jj,iitr);  % Desired control direction
end

% VelCtrl(iitr,:) = vdGain .* sqrt(sum(ctrl.^2, 1)); % Desired speed
VelCtrl(iitr,:) = Kp .* sum(ctrl.^2, 1) - Kd .* VelWorldFilt(iitr,:) ;  % Desired speed

% Speed of Sphero
vel = 0.1 * VelCtrl(iitr,j);            % Desired speed
VelInput(itr,j) = vel;                       
vel = min(vel,vMax);                    % Limit speed to maximum allowed velocity
vel = max(vel,0.01);                    % Keep speed > 0 
VelSatInput(itr,j) = vel;               % Saturated speed

thtC = atan2d(ctrl(2,j), ctrl(1,j));    % Control vector angle in world coordinate frame
ThtCtrl(iitr,j) = thtC;

ThtInput(itr,j) = wrapTo180( ThtCtrl(iitr,j) - Theta0(j) );

if itr > 1
    dT = Time(iitr) - Time(iitr-1);
    Omega(iitr,j) = wrapTo180( ThtInput(itr,j) - ThtInput(itr-1,j) ) ./ dT; % Angular velocity input
end

% Issue input command   
roll(Sph{j}, VelSatInput(itr,j), -ThtInput(itr,j) );


%%


SpheroState.VelCtrl     = VelCtrl;      % Speed from control vector
SpheroState.ThtCtrl     = ThtCtrl;      % Heading from control vector
SpheroState.VelInput    = VelInput;     % Input speed
SpheroState.VelSatInput = VelSatInput;  % Input speed
SpheroState.ThtInput    = ThtInput;     % Input heading
SpheroState.Omega       = Omega;        % Angular velocity 
SpheroState.Ctrl.posDes = posDes;       % Desired position


















































































































