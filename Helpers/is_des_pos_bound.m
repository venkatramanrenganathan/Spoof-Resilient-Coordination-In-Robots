function [iswithin] = is_des_pos_bound(frame_size, pos_des, thresh, Calib, R, T)
% bounds the desired pos to the internal region of the polygon world frame
% Inputs:
%   frame_size: size of camera frame [max_x; max_y]
%   pos_des:    desired position in world frame [[x1;y1], [x2;y2], ... ]
%   thresh:     threshold away from edge of frame in pixels
%   Calib:      camera calibration matrix
%   R:          extrinsic rotation matrix
%   T:          extrinsic translation vector
% Outputs:
%   iswithin:   vector with 1 at index of pos internal to the shape and 0
%               when the pos is external
%
% camera frame data will be indicated with '_c' at its end
% world frame data will be indicated with '_w' at its end

%% determining thresholded boundary corners in camer frame
%                   y
%   p4-----p3       ^
%   |      |        |
%   P1-----p2       |----> x
%
x_max = frame_size(1);
y_max = frame_size(2);
p_edge_c = [thresh, x_max-thresh, x_max-thresh, thresh; ...
            thresh, thresh,       y_max-thresh, y_max-thresh];

%% determining p_edge_c in world frame
imPts = [p_edge_c; ones(1,4)];
p     = Calib \ imPts;                    % Homogenious point coordinates on the image
Nc    = R * [0; 0; 1];                    % Normal of the plane (given in the camara coordinate frame)
d     = Nc.' * T;                         % Distance from camera to the plane
Nxp   = Nc.' * p;
P     = bsxfun(@rdivide, d*p, Nxp);       % Point coordinates (in camera coordinate frame)
Pw    = bsxfun(@plus, R.'*P, -R.'*T);     % Point coordinates (in world coordinate frame)
p_edge_w = Pw(1:2,:);
p_edge_w(1,:) = -p_edge_w(1,:);           % Orient the world frame s.t. z-axis is pointing up

%% determining if the points of pos_des (in world) are withing the polygon
% with corners p_edge_w
% the shape is convex and can be split into two convex trianlges
% a point p inside either one can be written as a linear combination of the
% corners such that the sum of the coefficients is 1 and the coefficiens
% are positive
[~,num_pts] = size(pos_des);                    % number of points being processed
p_edge_1_w = [p_edge_w(:,1:3); ones(1,3)];      % corners of first triangle
p_edge_2_w = [p_edge_w(:,[1,3,4]); ones(1,3)];  % corners of complementary triangle
B = [pos_des; ones(1,num_pts)];                 % all desired points
alphas_1 = p_edge_1_w\B;                        % solve p_edge * alphas = B twice
alphas_2 = p_edge_2_w\B;                        % where alpha is coefficients
ispos_1 = alphas_1 >= 0;                        % find positive alphas
ispos_2 = alphas_2 >= 0;                        % find positive alphas
iswithin = zeros(1,num_pts);                    % initialize iswithin
for i = 1 : num_pts
    % check if either top or bottom triple is all postive coefficients
    iswithin(i) = all(ispos_1(:,i)) | all(ispos_2(:,i)); 
end
% iswithin now indicates the points in pos_des that are internal to the
% shape


