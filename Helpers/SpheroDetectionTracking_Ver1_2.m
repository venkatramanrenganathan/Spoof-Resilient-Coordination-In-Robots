%%%%%%%%%%%%%%%%%%%%%
% 3D reconstruction %
%%%%%%%%%%%%%%%%%%%%% 
function SpheroState = SpheroDetectionTracking_Ver1_2(iitr, SpheroState, CameraParam)

col       = CameraParam.col;                % Detection color
cam       = CameraParam.cam;                % Camera object
Calib     = CameraParam.Calib;              % Camera calibration parameters
Rot       = CameraParam.Rot;                % Rotation from camera to world frame
Tran      = CameraParam.Tran;               % Translation from camera to world frame
% Norm    = CameraParam.Norm;              % Normal to checkerboard

numRob    = SpheroState.numRob;             % Number of robots

Sph       = SpheroState.Sph;                % Sphero objects
PosWorld  = SpheroState.PosWorld;           % Positions in world coordinate frame
PosPixel  = SpheroState.PosPixel;           % Positions in pixel
Bboxes    = SpheroState.Bboxes;             % Bounding boxes


if iitr == 1 % In the first iteration    
    
[posPixOrder, bboxesOrder, InitFrames] = ...
    Detect_Sphero_Initial_Ver2_3(cam, Sph, numRob, col);   % Detect Sphero locations in the order of sph vector

SpheroState.Video.InitFrames = InitFrames;

else % Other iterations  
    
[posPixAll, bboxes, frame] = Detect_Sphero_Ver2_4(cam, numRob); % Detect Spheros   

SpheroState.Video.Frames{iitr} = frame;

posOld = PosPixel(:,:,iitr-1);
posNew = posPixAll;
[posPixOrder, idx] = track_Sphero_Ver2_1(posOld, posNew); % Track Spheros  
bboxesOrder        = bboxes(:,idx); 
    
end

PosPixel(:,:,iitr) = posPixOrder;         % Robot positions on the image (in pixels)
Bboxes(:,:,iitr)   = bboxesOrder;         % Bounding boxes

% x-y position of robots in the world coordinate frame (defined via the checkerboard)
imPts = [posPixOrder; ones(1,numRob)];
p     = Calib \ imPts;                         % Homogenious point coordinates on the image
Nc    = Rot * [0; 0; 1];                       % Normal of the plane (given in the camara coordinate frame)
d     = Nc.' * Tran;                           % Distance from camera to the plane
Nxp   = Nc.' * p;
P     = bsxfun(@rdivide, d*p, Nxp);            % Point coordinates (in camera coordinate frame)
Pw    = bsxfun(@plus, Rot.'*P, -Rot.'*Tran);   % Point coordinates (in world coordinate frame)
posWorldOrder      =  Pw(1:2,:);
posWorldOrder(1,:) = -posWorldOrder(1,:);      % Orient the world frame s.t. z-axis is pointing up
PosWorld(:,:,iitr) =  posWorldOrder;           % Robot positions in 3D (world coord frame)    

%%

SpheroState.PosWorld = PosWorld;
SpheroState.PosPixel = PosPixel;
SpheroState.Bboxes   = Bboxes;











































































































