function [bound_pos_des] = find_bound_pos_des(dist, pos_cur, pos_lead, ...
                                              frame_size, thresh, Calib, ...
                                              R, T, incrmt)
% finds pos_des that is internal to the convex frame
% Inputs:
%   dist:       distance between two consecutive robots
%   pos_cur:    current position of the robots in world fram ([x1;y1] [x2;y2] ...)
%   pos_lead:   position of the leader in world frame (x;y)
%   frame_size: size of camera frame [max_x; max_y]
%   thresh:     threshold away from edge of frame in pixels
%   Calib:      camera calibration matrix
%   R:          extrinsic rotation matrix
%   T:          extrinsic translation vector
%   inc:        angle increment size
% Outputs:
%   bound_pos_des:  pos_des, but bounded to world frame seen by camera
%                   [[x1;y1], [x2;y2], ...]
% How it functions:
% - Get pos_des
% - Test if all points are internal
% - if internal --> Done 
%   else rotate first follower along a circle centered at the leader by an
%   angle 'ang' and repeat the loop until pos_des points are internal

[pos_des] = find_pos_des(dist, pos_cur, pos_lead);
[iswithin] = is_des_pos_bound(frame_size, pos_des, thresh, Calib, R, T);

if all(iswithin)
    bound_pos_des = pos_des;
    return;
else
%     r = norm (pos_lead - pos_cur(:,1)); % distance between leader and first follower
    tht_init = atan2d(-pos_lead(2)+pos_cur(2,1), -pos_lead(1)+pos_cur(1,1)); % angle between follower and leader with respect to the x-axis
    % switch between cw and ccw adding an angle to the tht_init and
    % simulating if the all pos_des would be inside the camera frame
    
  
    num_inc = ceil(180/incrmt); % find number of increments on each side of the leader-1st follower line
    for itr = 1 : num_inc*2 -1 % loop thru all cw and ccw increments to tht_init
        tht = tht_init + (-1)^(itr+1) * ceil(itr/2)*incrmt; % if tht_init = 0 --> tht= incrmt, -incrmt, 2*incrmt, -2*incrmt, ...
        pos_cur_temp = pos_cur;
        pos_cur_temp(1,1) = dist*cosd(tht) + pos_lead(1);
        pos_cur_temp(2,1) = dist*sind(tht) + pos_lead(2);
        [pos_des] = find_pos_des(dist, pos_cur_temp, pos_lead);
        [iswithin] = is_des_pos_bound(frame_size, pos_des, thresh, Calib, R, T);
        if all(iswithin)
            bound_pos_des = pos_des;
            return;
        end
    end
    
%     for ang = tht_init : incrmt : tht_init+360
%         pos_cur_temp = pos_cur;
%         pos_cur_temp(1,1) = dist*cosd(ang) + pos_lead(1);
%         pos_cur_temp(2,1) = dist*sind(ang) + pos_lead(2);
%         [pos_des] = find_pos_des(dist, pos_cur_temp, pos_lead);
%         [iswithin] = is_des_pos_bound(frame_size, pos_des, thresh, Calib, R, T);
%         if all(iswithin)
%             bound_pos_des = pos_des;
%             return;
%         end
%     end
    
    disp('Not Possible')
    bound_pos_des = pos_des;

    % TODO: if the look does not break --> bound_pos_des not found ... deal with it
end
