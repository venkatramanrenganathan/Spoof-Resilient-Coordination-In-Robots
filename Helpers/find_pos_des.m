function [pos_des] = find_pos_des(dist, pos_cur, pos_lead)
% pos_des: desired position of the robots ([x1;y1] [x2;y2] ...)
% dist: distance between two consecutive robots
% pos_cur: current position of the robots ([x1;y1] [x2;y2] ...)
% pos_lead: position of the leader (x;y)

[~,num_rob] = size(pos_cur);
pos_des = zeros(2,num_rob);
for j = 1 : num_rob
    if j == 1
        pos_cur_i = pos_cur(:,j);
        m = (pos_lead(2) - pos_cur_i(2))/(pos_lead(1) - pos_cur_i(1));
        if isinf(m) % if the robots are vertical
            pos_des_1 = [pos_lead(1), pos_lead(2) + dist];
            pos_des_2 = [pos_lead(1), pos_lead(2) - dist];
        else
            pos_des_1 = [pos_lead(1) + dist / sqrt(m^2+1), ...
                            pos_lead(2) + m * dist / sqrt(m^2+1)];
            pos_des_2 = [pos_lead(1) - dist / sqrt(m^2+1), ...
                            pos_lead(2) - m * dist / sqrt(m^2+1)];
        end
        dist1 = norm(pos_des_1 - pos_cur_i, 2);
        dist2 = norm(pos_des_2 - pos_cur_i, 2);
        if dist1 < dist2
            pos_des(:,j) = pos_des_1;
        else
            pos_des(:,j) = pos_des_2;
        end
    else
        if j == 2
            i_1 = pos_des(:,j-1);
            if isinf(m) % if the robots are vertical
              pos_des_1 = [i_1(1), i_1(2) + dist];
              pos_des_2 = [i_1(1), i_1(2) - dist];
            else
                pos_des_1 = [i_1(1) + dist / sqrt(m^2+1), ...
                             i_1(2) + m * dist / sqrt(m^2+1)];
                pos_des_2 = [i_1(1) - dist / sqrt(m^2+1), ...
                             i_1(2) - m * dist / sqrt(m^2+1)];
            end
            diff1 = norm(pos_des_1 - pos_lead, 2);
            diff2 = norm(pos_des_2 - pos_lead, 2);
            if diff1 < diff2
                pos_des(:,j) = pos_des_2;
            else
                pos_des(:,j) = pos_des_1;
            end

        else
            i_1 = pos_des(:,j-1);
            if isinf(m) % if the robots are vertical
                pos_des_1 = [i_1(1), i_1(2) + dist];
                pos_des_2 = [i_1(1), i_1(2) - dist];
            else
                pos_des_1 = [i_1(1) + dist / sqrt(m^2+1), ...
                             i_1(2) + m * dist / sqrt(m^2+1)];
                pos_des_2 = [i_1(1) - dist / sqrt(m^2+1), ...
                             i_1(2) - m * dist / sqrt(m^2+1)];  
            end
            diff1 = norm(pos_des_1 - pos_des(:,j-2), 2);
            diff2 = norm(pos_des_2 - pos_des(:,j-2), 2);
            if diff1 < diff2
                pos_des(:,j) = pos_des_2;
            else
                pos_des(:,j) = pos_des_1;
            end
        end
    end
end

