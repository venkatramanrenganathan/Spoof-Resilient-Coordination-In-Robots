function SpheroState = SpheroVideoStream_Ver1_2(iitr, SpheroState, CameraParam)

PosWorld    = SpheroState.PosWorld;     % Positions in world coordinate frame
PosPixel    = SpheroState.PosPixel;     % Positions in pixels
PosKalm     = SpheroState.PosKalm;      % Position of robots from Kalman filter(in world frame)
ThtKalm     = SpheroState.ThtKalm;      % Estimated headings from Kalman filter
MotionIndex = SpheroState.MotionIndex;  % Index to identify motion
ThtEst      = SpheroState.ThtEst;       % Estimated heading
ThtCtrl     = SpheroState.ThtCtrl;      % Control direction
Bboxes      = SpheroState.Bboxes;       % Bounding boxes
Frames      = SpheroState.Video.Frames; % Webcam images
numRob      = SpheroState.numRob;       % Number of robots
numItr      = SpheroState.numItr;       % Number of iterations

rec         = SpheroState.Video.Record; % rec = 'false' or 'true'
vid         = SpheroState.Video.vid;    % Handel to webcam image video
vid2        = SpheroState.Video.vid2;   % Handel to 3D reconstruction plot

Rot         = CameraParam.Rot;          % Rotation between image and world frames   
Tran        = CameraParam.Tran;         % Translation between image and world frames

%% Plot parameters

showHead    = true;     % Show heading vector in 3D reeconstruction plot
showCtrl    = true;     % Show control vector in 3D reeconstruction plot
tagFontSize = 20;       % Font size for tags shown in 3D recons. plot
ballSize    = 200;      % size of sphere in 3D recons. plot
arrowSize   = 100;      % size of aroow in 3D recons. plot


%% Display webcam image    

if (iitr == 1)   
    if rec, open(vid); end
    hdlImageFig   = figure;
    hdlImageAx = gca; 
    SpheroState.Video.hdlImage   = hdlImageFig;
    SpheroState.Video.hdlImageAx = hdlImageAx;
else    
    hdlImageFig   = SpheroState.Video.hdlImage;
    hdlImageAx = SpheroState.Video.hdlImageAx;
end

frame = Frames{iitr};

axes(hdlImageAx);
imshow(frame);
hold on;
scatter(PosPixel(1,:,iitr) ,PosPixel(2,:,iitr), 'filled', 'LineWidth', 2); % display center on image
for j = 1 : numRob
    rectangle('Position',Bboxes(:,j,iitr),'LineWidth',1,'EdgeColor',[0 0 1]);
    text(PosPixel(1,j,iitr),PosPixel(2,j,iitr), ['   ',num2str(j)], ...
        'FontSize',13, 'Color', [1 1 1]);
end
hold off;

drawnow;

if rec, writeVideo(vid, getframe(hdlImageFig)); end % Record video


%% Plot 3D reconstruction 

if (iitr == 1)   
    if rec, open(vid2); end
    hdlReconst   = figure;
    hdlReconstAx = gca;
    SpheroState.Video.hdlReconst   = hdlReconst;
    SpheroState.Video.hdlReconstAx = hdlReconstAx;
else    
    hdlReconst   = SpheroState.Video.hdlReconst;
    hdlReconstAx = SpheroState.Video.hdlReconstAx;
end

pos = PosWorld(:,:,iitr);                           % Current positions    
H = [cosd(ThtEst(iitr,:));  sind(ThtEst(iitr,:))];  % Heading vectors
C = [cosd(ThtCtrl(iitr,:)); sind(ThtCtrl(iitr,:))]; % Control vectors
    
% Plot points
axes(hdlReconstAx);    
hold on;
for j = 1 : numRob
    scatter3(pos(1,j),pos(2,j),0,ballSize,'fill');
    text(pos(1,j),pos(2,j),0,['  ',num2str(j)], 'FontSize', tagFontSize);
end

% Plot camera
plotCamera('Location',-Tran.'*Rot,'Orientation',Rot,'Opacity',0, 'Size', 50);
hold off;
viewMat = [0, 90];
view(viewMat);
axis equal
grid on
% set(gca,'CameraUpVector',[0 0 -1]);
% camorbit(gca,110,60);
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');

% Heading arrows
if showHead
    Vh = H .* arrowSize;
    hold on
    for j = 1 : numRob
        % Red color for heading
        plot3([pos(1,j);pos(1,j)+Vh(1,j)], [pos(2,j);pos(2,j)+Vh(2,j)], [0;0], ...
            'Color', [0.9 0.0 0.0], 'LineWidth', 3);
    end
    hold off
end
        
% Control arrows
if showCtrl
    Vc = C .* arrowSize*0.5;
    hold on
    for j = 1 : numRob
        % Blue color for control
        plot3([pos(1,j);pos(1,j)+Vc(1,j)], [pos(2,j);pos(2,j)+Vc(2,j)], [0;0], ...
            'Color', [0.0 0.0 0.9], 'LineWidth', 3);
    end
    hold off
end

drawnow;

if rec, writeVideo(vid2, getframe(hdlReconst));  end % Record video
    
% keyboard;


%%

if iitr == numItr*numRob
    % Stop recording
    if rec, close(vid); end
    if rec, close(vid2); end
end































































































