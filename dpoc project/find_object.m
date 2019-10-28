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
