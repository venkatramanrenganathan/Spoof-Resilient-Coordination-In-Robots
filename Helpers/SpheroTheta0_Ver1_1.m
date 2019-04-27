function theta0 = SpheroTheta0_Ver1_1(j, SpheroStateOrig, CameraParam)

SpheroState        = SpheroStateOrig;    % A copy of the original SpheroState
Sph                = SpheroState.Sph(j); % Desired Sphero
SpheroState.Sph    = Sph;
SpheroState.numRob = 1;                  % Number of robots set to 1

numItr             = 100;                % Maximum number of iterations
SpheroState.numItr = numItr;          
distPix  = 80;                           % Desired distance to be travelled in pixels
vel                = 40;                 % Moving forward speed
ang                = 0;                  % Moving forward angle


%%

roll(Sph{1},0,0);  % Reset headings to zero

SpheroState = SpheroLoadParam_Ver1_2(SpheroState);  % Load parameters

for itr = 1 : numItr
    
    % Move Sphero forward
    disp('Moving forward...');        
    roll(Sph{1},vel,ang);
    
    % 3D reconstruction  
    disp('Image detection & tracking');
    SpheroState = SpheroDetectionTracking_Ver1_2(itr, SpheroState, CameraParam);
    
    % Heading and speed estimation
    disp('Heading estimation');
    SpheroState.Time(itr)  =  cputime;
    SpheroState = SpheroHeadingSpeedEstim_Ver1_2(itr, SpheroState);

    % Stop Sphero once desired distance is travelled
    if norm(SpheroState.PosPixel(:,:,itr) - SpheroState.PosPixel(:,:,1)) >= distPix        
        roll(Sph{1},0,ang);
%         pause(0.5);
        
        % Find theta0
        ThtEst  = SpheroState.ThtEst;
        theta0  = nanmean(ThtEst - ang); 
        
        fprintf('\n\ntheta0 estimated.\n\n')
        break;
    end
        
    
end
    
    
    
    
    
    
    



















































































































