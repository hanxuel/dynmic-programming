function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

% put your code here

Nx = size(stateSpace,1);
Nu = size(controlSpace,1);
[~, gate_index, ] = intersect(stateSpace,gate,'rows');
gamap = 0.5;
pool_num_time_steps = 4;            % number of time steps that pass if moving into pool or pond
detected_additional_time_steps = 6;	% additional time steps if detected and brought to gate


for k = 1: Nu
    G(:,k) = Inf(Nx,1);
    action = controlSpace(k);
    for i = 1:Nx
         x = stateSpace(i,:);
         y = move(x,action);
         [~, j, ] = intersect(stateSpace,y,'rows');
         
         % over the map
         if isempty(j)
             continue
         end
             
         if action == 'p'
             [camera_nob,d] = find_object(x,cameras,map,false);
             [~,d2] = find_object(x,mansion,map,true);
             if isempty(d2)
                 P_success = 0.001;
             else
                P_success = max(0.001,gamap/d2);
             end
             P_notsuccess = 1 - P_success;
             P = 1;
             for c_i = 1: length(camera_nob)
                 P = (1 - cameras(camera_nob(c_i),3)/d(c_i))*P;
             end
             G(i,k) = 1+P_notsuccess*(1-P)*detected_additional_time_steps;
%              if j == gate_index
%                 P(i,gate_index,k) = 1;
%              else
%                  P(i,gate_index,k) = 1-P(i,j,k);
%              end
%              if j ~= gate_index
%                 P (i,j,k) = P_notsuccess *P(i,j,k);
%              end
%              P(i,gate_index,k) = P_notsuccess * P(i,gate_index,k);
%              
             
         

         else
             [camera_nob,d] = find_object(y,cameras,map,false);
             P= 1;
             for c_i = 1: length(camera_nob)
                 P = (1 - cameras(camera_nob(c_i),3)/d(c_i))*P;
             end
%              if j == gate_index
%                 P= 1;
%              else
%                  P = 1-P;
%              end
             if map(y(2),y(1)) < 0 && map(x(2),x(1))>=0
                P = P^4; 
            	G(i,k) = pool_num_time_steps+(1-P)*detected_additional_time_steps;
             else
                G(i,k) = 1+(1-P)*detected_additional_time_steps;
             end
             
             
         end
    
    end
end
end

function y = move(x,action)
if action == 'n'
    y = [x(1),x(2)+1];
elseif action == 'w'
    y = [x(1)-1,x(2)];
elseif action == 'e'
    y = [x(1)+1,x(2)];
elseif action == 's'
    y = [x(1),x(2)-1];
else
    y = x;
end

end

function d = get_distance(y1,y2)
    if y1(1) == y2(1)
        d = abs(y2(2)-y1(2));
    else
        d = abs(y2(1)-y1(1));
    end
    
end


% compute related camera
function [camera_nob,d] = find_object(y,cameras,map,isman)
column_camera = find(cameras(:,1) == y(1));
col_camera_nob = [];
d = [];
for a = 1:length(column_camera)
     camera1_y = cameras(column_camera(a),2);
     % judge obstacles
     index = 2*(y(2)<camera1_y)-1;
     middle = map(y(2):index:camera1_y,y(1));
     if isempty(find(middle(1:end-1)>0,1))
         col_camera_nob = [col_camera_nob,column_camera(a)];
     end
end                    
    
row_camera = find(cameras(:,2) == y(2));
row_camera_nob = [];
for a = 1:length(row_camera)
     camera1_x = cameras(row_camera(a),1);
     % judge obstacles
     index = 2*(y(1)<camera1_x)-1;
     middle = map(y(2),y(1):index:camera1_x);
     if isempty(find(middle(1:end-1)>0,1)) 
         row_camera_nob = [row_camera_nob,row_camera(a)];
     end
end   
camera_nob= unique([col_camera_nob,row_camera_nob]);
for a = 1: length(camera_nob)
    d(a) = get_distance(cameras(camera_nob(a),1:2),y);
end
if isman == true
    d = min(d);
end
    
end










