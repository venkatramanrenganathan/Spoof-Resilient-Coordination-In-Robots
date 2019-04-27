% Version 1_1:
%
% -
%
addpath(genpath('J:\Hubic\Matlab\Formation Control\13 - Sphero Formation\Sphero'));
addpath('Helpers');
addpath('EKF')
addpath(genpath('Calibration Images'));


%% Preallocate parameters

SphNames = {'RPY', 'WYW','OWY'};                % Name of spheros
% 'YWB', 'WWP', 'WYP' ,'OWY', 'OGY', 'YWR', 'WOR', 'OYR', 'ROY', 'RPY'
CameraParam.col = [255,   0,   0]; % Detection color for followers (red)
SpheroState.SphNames = SphNames;

numItr             = 10;         % Number of iterations to keep the data (must be > 1)
SpheroState.numItr = numItr;


%% Test Webcam

% camList = webcamlist;  % Identify Available Webcams
if isfield(CameraParam,'cam')
    delete(CameraParam.cam)
    CameraParam = rmfield(CameraParam,'cam');
end
cam = webcam(1);         % Connect to the webcam
preview(cam);            % Preview Video Stream

clear cam                % Release webcam 


%% Connect to Spheros

clc
SpheroState = SpheroConnect_Ver1_1(CameraParam, SpheroState);
numRob      = SpheroState.numRob;


%% Record movie (Video settings)

SpheroState.Video.VideoName = 'Test04';
SpheroState.Video.Record    = false; % ture;

SpheroState = SpheroVideoSetup_Ver1_0(SpheroState);


%% Initialize camera and detect checkerboard

CameraParam.squareSize = 28; % Checkerboard square size in mm
CameraParam.paramFile  =  'CameraParams_Logitech_640x480_Gans.mat';

CameraParam = CameraCheckerboard(CameraParam);


%% Theta0 estimation

clc
SpheroState.Theta0 = SpheroTheta0_Ver1_3(SpheroState, CameraParam);     


%% Leader robot

CameraParamLeader     = CameraParam;
CameraParamLeader.col = [0,   255,   255]; % Detection color for leader (blue)

SpheroStateLeader.numRob = 1;
SpheroStateLeader.numItr = numItr;
SpheroStateLeader = SpheroLoadParam_Ver1_3S(SpheroStateLeader);
SpheroStateLeader.Sph = {};

%% Formation control

clc
close all

% Preallocate required variables
SpheroState = SpheroLoadParam_Ver1_3S(SpheroState);

itr = 0;
while  true
itr = itr + 1;    

j = 0;
while j <= numRob-1
j = j + 1;    
iitr = (itr-1)*numRob + j; % Current iteration number

% Sphero detection, tracking, and 3D reconstruction  
disp('Image detection & tracking');
SpheroState = SpheroDetectionTracking_Ver1_4(iitr, SpheroState, CameraParam);

% Reset detection when it fails
if any(isnan(SpheroState.PosPixel(:,:,iitr))), itr = 1; j = 0; close all; continue; end

% Leader detection 
if iitr > 1
disp('Leader detection & tracking');
SpheroStateLeader = SpheroDetectionTracking_Ver1_4(iitr, SpheroStateLeader, CameraParamLeader);
end

% Heading and speed estimation
disp('Heading and speed estimation');
SpheroState.Time(iitr)  =  cputime;
SpheroState = SpheroHeadingSpeedEstim_Ver1_2(iitr, SpheroState);

% Formation control  
disp('Control');
SpheroState = SpheroRendezvousControl_Ver1_1(iitr,itr,j, SpheroState, SpheroStateLeader, CameraParam);

% Kalman Filter
disp('Kalman filter');
SpheroState = SpheroKalmanFilter_Ver1_2(iitr, SpheroState);

% Display video stream
SpheroState = SpheroVideoStream_Ver1_3(iitr, SpheroState, CameraParam);

% Keep only up to the last 'numItr' number of data
SpheroState = SpheroShiftData_Ver1_0(itr, SpheroState);
if itr == numItr, itr = itr - 1; end

fprintf('\n\n\n')
% keyboard; % dbcont;

end
end









%% Stop Spheros

SpheroStopVideo(SpheroState);  % Close and save video

for j = 1 : numRob
    brake(SpheroState.Sph{j});     % Stop Spheros
    roll(SpheroState.Sph{j},0,0);  % Reset orientation
end
    
%% Disconnect from Spheros

SpheroDisconnect_Ver1_1(SpheroState);


%% Reset headings to zero

% for j =  1 : numRob
%     roll(SpheroState.Sph{j},0,0); % Reset headings to zero
% end

%% Save variables

% if ~exist(Name)
%     Name = 'Sphero_v7_01_vid_01';
% end
% currentFolder = pwd;
% address =  strcat(currentFolder,'/SavedData/');
% fileName = strcat(Name, '_data');
% fileType = '.mat';
% fullAddress = strcat(address,fileName,fileType);
% 
% % Save all variables
% save(fullAddress)
























































