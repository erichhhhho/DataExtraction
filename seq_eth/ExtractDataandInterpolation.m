%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script to preprocess the eth data
%
% World coordinate data:[FrameID PedID y x](world_coordinate.csv)
% World coordinate normalized data(by min and max):[FrameID PedID y x](world_coordinate_normalized.csv)
% World interpolated coordinate data(by min and max):[FrameID PedID y x](world_coordinate_inter.csv)
% World interpolated coordinate normalized data(by min and max):[FrameID PedID y x](world_coordinate_inter_normalized.csv)
%
% Velocity data(m/s):[FrameID PedID v_y v_x](velocity.csv)
% Pixel coordinate data:[FrameID PedID v u](pixel.csv)
% Pixel coordinate normalized data(by width and height):[FrameID PedID v u](pixel_normalized.csv)
% other:[ymax ymin vmax vmin
%        xmax xmin umax umin](other.csv)
% 
% by He Wei, Oct 31, 2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

H = dlmread('H.txt');

data = dlmread('obsmat.txt');

% pos = [xpos ypos 1]
pos = [data(:,3) data(:,5) ones(size(data,1),1)];
%velocity=[frameID PedID v_y v_x]
velocity = [data(:,1)'; data(:,2)';data(:,8)'; data(:,6)'];
%world_coordinate=[frameID PedID y x]
world_coordinate = [data(:,1)'; data(:,2)';data(:,5)' ;data(:,3)'];
% Assuming H converts world coordinates to image coordinates
pixel_pos_unnormalized_pinH = pinv(H) * pos';

pixel_pos_unmormalized_H=pos/H';

% Normalize pixel pos
% Each column contains [u; v] 
pixel_pos_normalized_pinH = bsxfun(@rdivide, pixel_pos_unnormalized_pinH([1,2],:), ...
                              pixel_pos_unnormalized_pinH(3,:));

% Each column contains [u; v] 
pixel_pos_normalized_H = pixel_pos_unmormalized_H(:,1:2)./ repmat( pixel_pos_unmormalized_H(:,3),[1 2]);
pixel=  [data(:,1)'; data(:,2)';pixel_pos_normalized_H'];                     
       
pixel_pos_frame_normalized_pinH=[data(:,1)'; data(:,2)';pixel_pos_normalized_pinH];
% Add frame number and pedestrian ID information to the matrix
pixel_pos_frame_ped_pinH = [data(:,1)'; data(:,2)';pixel_pos_normalized_pinH];

pixel_normalized = [data(:,1)'; data(:,2)';pixel_pos_normalized_H'];

pixel_pos_frame_ped_pinH(3,:) = pixel_pos_frame_ped_pinH(3,:) / 480;
pixel_pos_frame_ped_pinH(4,:) = pixel_pos_frame_ped_pinH(4,:) / 640;

pixel_normalized(3,:) = (2*pixel_normalized(3,:) / 480)-1;
pixel_normalized(4,:) = (2*pixel_normalized(4,:) / 640)-1;

pixel_normalized01=[data(:,1)'; data(:,2)';(pixel(3,:) / 576);(pixel(4,:) / 720)];

% other=[max(world_coordinate(3,:)) min(world_coordinate(3,:)) max(pixel(3,:))  min(pixel(3,:));
%        max(world_coordinate(4,:)) min(world_coordinate(4,:)) max(pixel(4,:))  min(pixel(4,:))];

% y_normalized=2*((world_coordinate(3,:)-min(world_coordinate(3,:))) / (max(world_coordinate(3,:))- min(world_coordinate(3,:))))-1;
% x_normalized=2*((world_coordinate(4,:)-min(world_coordinate(4,:))) / (max(world_coordinate(4,:))- min(world_coordinate(4,:))))-1;
[Y,PSY]=mapminmax(world_coordinate(3,:),-1,1);
[X,PSX]=mapminmax(world_coordinate(4,:),-1,1);

[pixel_v,PSV]=mapminmax(pixel(3,:),-1,1);
[pixel_u,PSU]=mapminmax(pixel(4,:),-1,1);



world_coordinate_normalized=[data(:,1)'; data(:,2)';Y;X];

%Interpolation
world_coordinate_inter=[];

for pednum=1:max(world_coordinate(2,:))
    ped=[];
    for point=1:length(world_coordinate(1,:))
    
       if world_coordinate(2,point)==pednum
          ped=[ped world_coordinate(:,point)];
        
       end
    end
    
    if ~isempty(ped)&& length(ped(1,:))>1
       
        time=[];
        time= min(ped(1,:)):max(ped(1,:));
        x_inter = interp1(ped(1,:),ped(4,:),time);
        y_inter = interp1(ped(1,:),ped(3,:),time);


        ped=[time;repmat(pednum,1,length(time));y_inter;x_inter];


        world_coordinate_inter=[world_coordinate_inter ped];
    end
end
    
world_coordinate_inter = sortrows(world_coordinate_inter',1);
world_coordinate_inter = world_coordinate_inter';

world_coordinate_inter_cat = [world_coordinate_inter(4,:)' world_coordinate_inter(3,:)'  ones(length(world_coordinate_inter(4,:)),1)];
Pixel_inter = world_coordinate_inter_cat / H';

pixel_coordinate_inter=[world_coordinate_inter(1,:); world_coordinate_inter(2,:); round(Pixel_inter(:,1)'./Pixel_inter(:,3)');round(Pixel_inter(:,2)'./Pixel_inter(:,3)') ];


[Y_inter,PSY_inter]=mapminmax(world_coordinate_inter(3,:),-1,1);
[X_inter,PSX_inter]=mapminmax(world_coordinate_inter(4,:),-1,1);

[Y_inter01,PSY_inter01]=mapminmax(world_coordinate_inter(3,:),0,1);
[X_inter01,PSX_inter01]=mapminmax(world_coordinate_inter(4,:),0,1);

world_coordinate_inter_normalized=[world_coordinate_inter(1,:); world_coordinate_inter(2,:);Y_inter;X_inter];
world_coordinate_inter_normalized01=[world_coordinate_inter(1,:); world_coordinate_inter(2,:);Y_inter01;X_inter01];

pixel_coordinate_inter_normalized([1,2],:)=pixel_coordinate_inter([1,2],:);
pixel_coordinate_inter_normalized(3,:) = 2*(pixel_coordinate_inter(3,:) / 480)-1;
pixel_coordinate_inter_normalized(4,:) = 2*( pixel_coordinate_inter(4,:) / 640)-1;

pixel_coordinate_inter_normalized01([1,2],:)=pixel_coordinate_inter([1,2],:);
pixel_coordinate_inter_normalized01(3,:) = pixel_coordinate_inter(3,:) / 480;
pixel_coordinate_inter_normalized01(4,:) =  pixel_coordinate_inter(4,:)/ 640;

other=[PSY; PSX; PSY_inter;PSX_inter;PSY_inter01;PSX_inter01;PSV;PSU];

csvwrite('pixel_coordinate_inter.csv', pixel_coordinate_inter);

csvwrite('pixel_coordinate_inter_normalized.csv', pixel_coordinate_inter_normalized);

csvwrite('pixel_coordinate_inter_normalized01.csv', pixel_coordinate_inter_normalized01);

% Save the positions to a mat file
%save('pixel_pos_pinH.mat', 'pixel_pos_frame_ped_pinH');
%csvwrite('pixel_pos_pinH.csv', pixel_pos_frame_ped_pinH);

%save('pixel_normalized.mat', 'pixel_normalized');
csvwrite('pixel_normalized.csv', pixel_normalized);
csvwrite('pixel_normalized01.csv', pixel_normalized01);

%save('pixel.mat', 'pixel');
csvwrite('pixel.csv', pixel);

%save('pixel_pos_frame_normalized_pinH.mat', 'pixel_pos_frame_normalized_pinH');
%csvwrite('pixel_pos_frame_normalized_pinH.csv', pixel_pos_frame_normalized_pinH);

%save('velocity.mat', 'velocity');
csvwrite('velocity.csv', velocity);

%save('world_coordinate.mat', 'world_coordinate');
csvwrite('world_coordinate.csv', world_coordinate);

csvwrite('world_coordinate_normalized.csv', world_coordinate_normalized);

csvwrite('world_coordinate_inter.csv', world_coordinate_inter);

csvwrite('world_coordinate_inter_normalized.csv', world_coordinate_inter_normalized);

csvwrite('world_coordinate_inter_normalized01.csv', world_coordinate_inter_normalized01);

save('other.mat', 'other');


