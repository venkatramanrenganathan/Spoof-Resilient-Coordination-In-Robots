% Function spoofing_wmsr updates the information state of each
% vehicles after sorting & removing extreme values from its in-neighbors
%
function x = spoofing_wmsr_Ver1_2(x_0, par)


N = par.N;              % Number of nodes
F = par.F;              % Number of malicious nodes
D = par.D;              % Degree matrix
A = par.A;              % Adjacency matrix
idxMal = par.idxMal;    % Index of malicious agents
idxSpf = par.idxSpf;    % Index of spoofed nodes
L = D - A;              % Laplacian matrix

x = zeros(N, 1);        % State vector
 
% Do not update the state of malicious or spoofed agents
x(idxMal) = x_0(idxMal);
x(idxSpf) = x_0(idxSpf);


%%


for i = 1 : N  
    if ~( any(i == idxSpf) || any(i == idxMal) )
        L_i_row = L(i,:)';
        before_sort = [x_0 L_i_row];
        % Extract only in-neighbors
        condition = L_i_row >= 0;
        before_sort(condition,:) = [];  
        before_sort = before_sort(:,1);                      
        % removing larger values - sort descendingly
        ascend_sort = sortrows(before_sort, -1);              
        indices = find(ascend_sort > x_0(i));
        if(~isempty(indices))
            if(length(indices) > F)
                % if # of values larger than x(i) > F, delete F larger ones
                for j = 1:F
                    ascend_sort(indices(j),:) = [];
                end
            else
                % else delete all larger values
                ascend_sort(indices,:) = [];
            end
        end
        % removing smaller values          
        ascend_sort = sortrows(ascend_sort);
        indices = find(ascend_sort < x_0(i));
        if(~isempty(indices))
            if(length(indices) > F)
                for j = 1:F
                    ascend_sort(indices(j),:) = [];
               end
            else
                ascend_sort(indices,:) = [];
            end
        end
        remaining_count = length(ascend_sort);
        weight = 1/(remaining_count+1);
        x(i) = sum(weight*ascend_sort) + weight* x_0(i); 
        if(abs(x(i) - x_0(i)) > 1)
            if(x(i) > x_0(i))
                x(i) = x(i) - 0.75*abs(x(i) - x_0(i));
            else
                x(i) = x(i) + 0.75*abs(x(i) - x_0(i));
            end
        end
    end
end
    
    
    
    
end































