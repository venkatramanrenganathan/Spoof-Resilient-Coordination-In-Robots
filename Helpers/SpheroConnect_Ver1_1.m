function SpheroState = SpheroConnect_Ver1_1(CameraParam, SpheroState)


SphNames    = SpheroState.SphNames;
col         = CameraParam.col;         % Color used for detection
numRob      = length(SphNames);        % Number of robots

bklight  = 0;          % Back LED
MotionTO = 0.5;        % Motion timeout
hdshk    = 1;          % Bluetooth handshake
resTO    = 100;        % Response timeout


Sph = {};

for j = 1 : numRob
    
    varName  = strcat('Sphero-', SphNames{j});
    fprintf('Connecting to %s ...\n', varName);
    
    sph = sphero(varName);
    sph.InactivityTimeout = 60000;
    sph.Color = col;
    sph.BackLEDBrightness = bklight;
    sph.MotionTimeout = MotionTO;
    sph.Handshake = hdshk;  
    sph.ResponseTimeout = resTO;
    
    connect(sph);
    
    Sph{j} = sph;
    
    fprintf('Connected.\n\n');
    pause(0.5);
    
end


SpheroState.Sph     = Sph;     % Spheros
SpheroState.numRob  = numRob;  % Number of robots























































































