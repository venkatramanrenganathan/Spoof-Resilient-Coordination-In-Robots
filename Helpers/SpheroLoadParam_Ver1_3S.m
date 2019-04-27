function SpheroState = SpheroLoadParam_Ver1_3S(SpheroState)


numItr =  SpheroState.numItr;                        % Number of iterations
numRob =  SpheroState.numRob;                        % Number of iterations

SpheroState.Param.numRob          = numRob;          % Number of robots
SpheroState.Param.kv              = 0.5;             % Gain for V low-pass filter
SpheroState.Param.distPixelThresh = 5;               % Threshold for estimating the heading
SpheroState.Param.VelPixelThresh  = 10;              % Threshold to identify robots with large enough motion
SpheroState.Param.vMax            = 55;              % Maximum allowed velocity
SpheroState.Param.Kp              = 01;              % P-Gain for PID control
SpheroState.Param.Kd              = 5;               % D-Gain for PID control

% Kalman filter parameters
SpheroState.Param.Qkalm           = [1.0   0.0   0.0
                                     0.0   1.0   0.0
                                     0.0   0.0   1.0];     % Covariance of process
SpheroState.Param.Rkalm           = [0.1   0.0   0.0
                                     0.0   0.1   0.0
                                     0.0   0.0   0.1];     % Covariance of measurement 
SpheroState.Param.Pkalm           = repmat([0.1   0.0   0.0
                                            0.0   0.1   0.0
                                            0.0   0.0   1.5], 1,1,numRob);  % Initial state covraiance
     


% SpheroState.iitr        = iitr;
SpheroState.Time          = zeros(numItr*numRob, 1);         % CPU time at each iteration
SpheroState.PosWorld      = NaN(2,numRob, numItr*numRob);    % Position array of robots (in world frame)
SpheroState.PosKalm       = zeros(2,numRob, numItr*numRob);  % Position from Kalman filter (in world frame)
SpheroState.PosPixel      = NaN(2,numRob, numItr*numRob);    % Pixel position array of robots
SpheroState.Bboxes        = zeros(4,numRob, numItr*numRob);  % Bounding boxes for display
SpheroState.PosPixelAll   = cell(numItr*numRob, 1);          % Position of all detected objects (unordered)
SpheroState.BboxesAll     = cell(numItr*numRob, 1);          % Bounding boxes of all detected objects (unordered)

SpheroState.VelWorld      = zeros(numItr*numRob, numRob);    % Estimated speed in world coordinate frame
SpheroState.VelPixel      = zeros(numItr*numRob, numRob);    % Estimated speed in pixels
SpheroState.VelPixelFilt  = zeros(numItr*numRob, numRob);    % Low-pass filtered speed in pixels
SpheroState.VelWorldFilt  = zeros(numItr*numRob, numRob);    % Low-pass filtered speed in world coordinates
SpheroState.VelCtrl       = zeros(numItr, numRob);           % Desired speed from control command 
SpheroState.VelInput      = zeros(numItr, numRob);           % Speed input command 
SpheroState.VelSatInput   = zeros(numItr, numRob);           % Saturated speed input command 

SpheroState.MotionIndex   = false(numItr*numRob, numRob);    % Index to robots with large enough motion
SpheroState.ThtEst        = NaN(numItr*numRob,numRob);       % Estimated heading from image
SpheroState.ThtKalm       = zeros(numItr*numRob,numRob);     % Estimated headings from Kalman filter
SpheroState.ThtCtrl       = zeros(numItr*numRob, numRob);    % Desired angle from control command
SpheroState.ThtInput      = zeros(numItr, numRob);           % Heading angle input command
SpheroState.Omega         = zeros(numItr*numRob, numRob);    % Angular velocity 

SpheroState.Video.Frames  = cell(numItr*numRob,1);           % Video stream


% Leader-Follow Control Variables
SpheroState.Ctrl.dist           = 300;                              % Distance between two robots
SpheroState.Ctrl.frameSize      = [640; 480];                       % Size of camer frame in pixels
SpheroState.Ctrl.thresh         = 30;                               % Distance in pixels from edges of the camera frame in which the Spheros are banned
SpheroState.Ctrl.incrmt         = 10;                               % Increment angle in degrees (when searching for locations internal to the camera frame
SpheroState.Ctrl.posDes         = zeros(2,numRob, numItr*numRob);   % Desired postition for Spheros to go to

































