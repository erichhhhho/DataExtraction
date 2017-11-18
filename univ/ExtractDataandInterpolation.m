%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script to preprocess the ucy data/univ
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
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

H = dlmread('/media/hesl/OS/Documents and Settings/N1701420F/Desktop/pedestrians/ucy_crowd/data_students03/H.txt');

fid = fopen('../data_university_students/students003.vsp');

% First line contains the number of splines (or pedestrians?)
tline = fgetl(fid);
components = strsplit(tline, ' - ');
numSplines = str2num(cell2mat(components(1)));

pixel = [];


minDiff = Inf;
for spline=1:numSplines
   
    % First line for each spline contains the number of control
    % points
    tline = fgetl(fid);
    components = strsplit(tline, ' - ');
    numPoints = str2num(cell2mat(components(1)));
    oldframe=0;
    for point=1:numPoints                        
        % Each line contains the information x, y, frame_number,
        % gaze(not needed)
        tline = fgetl(fid);
        components = strsplit(tline, ' ');
        x = str2num(cell2mat(components(1)));
        y = str2num(cell2mat(components(2)));
        frame = str2num(cell2mat(components(3)));
        
        if point ~= 1
            minDiff = min(minDiff, frame-oldframe);
        end
        oldframe = frame;
        % Putting x, y in reverse order
        vec = [frame; spline; 288-y; x+360];
        %disp(vec(1))
        pixel = [pixel vec];
       
    end    
end
fprintf('The min diff between subsequent frames is %f\n', minDiff);



% NOTE x, y coordinates are recorded assuming center of the frame
% is (0,0)

[Y,I] = sort(pixel(1,:));
pixel_pos_frame_ped_sorted = pixel(:,I);
pixel = pixel_pos_frame_ped_sorted;
% 
% save('pixel_pos_frame_ped_oframeID.mat', 'pixel_pos_frame_ped_sorted');
% csvwrite('pixel_pos_frame_ped_oframeID.csv', pixel_pos_frame_ped_sorted);

%divided by height and width

pixel_pos_frame_ped_sorted(3,:) = pixel_pos_frame_ped_sorted(3,:) / 576;
pixel_pos_frame_ped_sorted(4,:) = pixel_pos_frame_ped_sorted(4,:) / 720;

% save('pixel_pos_oframeID.mat', 'pixel_pos_frame_ped_sorted');
% csvwrite('pixel_pos_oframeID.csv', pixel_pos_frame_ped_sorted);

%adjust the frameID


pixel_adjustedFrameID=repmat(pixel,1,1);
pixel_adjustedFrameID(1, :) = floor(pixel_adjustedFrameID(1, :) / 4);

%save('pixel.mat', 'pixel');
csvwrite('pixel.csv', pixel);
csvwrite('pixel_adjustedFrameID.csv', pixel_adjustedFrameID);

pixel_cat = [pixel(4,:)' pixel(3,:)'  ones(length(pixel(4,:)),1)];
pixel_adjustedFrameID_cat = [pixel_adjustedFrameID(4,:)' pixel_adjustedFrameID(3,:)'  ones(length(pixel_adjustedFrameID(4,:)),1)];

%P=[x,y,1]
P = pixel_cat * H';

P_adjustedFrameID =pixel_adjustedFrameID_cat * H';

%world_coordinate=[FrameID PedID Y X]
world_coordinate=[pixel([1,2],:);P(:,2)'; P(:,1)'];
world_coordinate_adjustedFrameID=[pixel_adjustedFrameID([1,2],:);P_adjustedFrameID(:,2)'; P_adjustedFrameID(:,1)'];


[Y,PSY]=mapminmax(world_coordinate(3,:),-1,1);
[X,PSX]=mapminmax(world_coordinate(4,:),-1,1);

[Y_adjustedFrameID,PSY_adjustedFrameID]=mapminmax(world_coordinate_adjustedFrameID(3,:),-1,1);
[X_adjustedFrameID,PSX_adjustedFrameID]=mapminmax(world_coordinate_adjustedFrameID(4,:),-1,1);

%world_coordinate_normalized=[Adjusted_FrameID PedID Y X]
world_coordinate_normalized=[pixel([1,2],:);Y;X];
world_coordinate_normalized_adjustedFrameID=[pixel_adjustedFrameID([1,2],:);Y_adjustedFrameID;X_adjustedFrameID];

pixel_normalized01([1,2],:)=pixel([1,2],:);
pixel_normalized01(3,:) = pixel(3,:) / 576;
pixel_normalized01(4,:) = pixel(4,:) / 720;

pixel_normalized([1,2],:)=pixel([1,2],:);
pixel_normalized(3,:) = 2*(pixel(3,:) / 576)-1;
pixel_normalized(4,:) = 2*(pixel(4,:) / 720)-1;

pixel_normalized_adjustedFrameID([1,2],:)=pixel_adjustedFrameID([1,2],:);
pixel_normalized_adjustedFrameID(3,:) = pixel(3,:) / 576;
pixel_normalized_adjustedFrameID(4,:) = pixel(4,:) / 720;

%save('pixel_normalized.mat', 'pixel_normalized');
csvwrite('pixel_normalized01.csv', pixel_normalized01);
csvwrite('pixel_normalized.csv', pixel_normalized);
csvwrite('pixel_normalized_adjustedFrameID.csv', pixel_normalized_adjustedFrameID);


%Interpolation under world coordinate
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
Pixel_inter = world_coordinate_inter_cat /(H');

pixel_coordinate_inter=[world_coordinate_inter(1,:); world_coordinate_inter(2,:); round(Pixel_inter(:,2)'); round(Pixel_inter(:,1)')];


[Y_inter,PSY_inter]=mapminmax(world_coordinate_inter(3,:),-1,1);
[X_inter,PSX_inter]=mapminmax(world_coordinate_inter(4,:),-1,1);

[Y_inter01,PSY_inter01]=mapminmax(world_coordinate_inter(3,:),0,1);
[X_inter01,PSX_inter01]=mapminmax(world_coordinate_inter(4,:),0,1);

world_coordinate_inter_normalized=[world_coordinate_inter(1,:); world_coordinate_inter(2,:);Y_inter;X_inter];
world_coordinate_inter_normalized01=[world_coordinate_inter(1,:); world_coordinate_inter(2,:);Y_inter01;X_inter01];

pixel_coordinate_inter_normalized([1,2],:)=pixel_coordinate_inter([1,2],:);
pixel_coordinate_inter_normalized(3,:) = 2*(pixel_coordinate_inter(3,:) / 576)-1;
pixel_coordinate_inter_normalized(4,:) = 2*( pixel_coordinate_inter(4,:) / 720)-1;

pixel_coordinate_inter_normalized01([1,2],:)=pixel_coordinate_inter([1,2],:);
pixel_coordinate_inter_normalized01(3,:) = pixel_coordinate_inter(3,:) / 576;
pixel_coordinate_inter_normalized01(4,:) =  pixel_coordinate_inter(4,:)/ 720;

%Adjust frameID Interpolation under world coordinate

world_coordinate_inter_adjustedFrameID=[];


for pednum=1:max(world_coordinate_adjustedFrameID(2,:))
    ped=[];
    for point=1:length(world_coordinate_adjustedFrameID(1,:))
    
       if world_coordinate_adjustedFrameID(2,point)==pednum
          ped=[ped world_coordinate_adjustedFrameID(:,point)];
        
       end
    end
    
    if ~isempty(ped)&& length(ped(1,:))>1
       
        time=[];
        time= min(ped(1,:)):max(ped(1,:));
        x_inter = interp1(ped(1,:),ped(4,:),time);
        y_inter = interp1(ped(1,:),ped(3,:),time);


        ped=[time;repmat(pednum,1,length(time));y_inter;x_inter];


        world_coordinate_inter_adjustedFrameID=[world_coordinate_inter_adjustedFrameID ped];
    end
end

world_coordinate_inter_adjustedFrameID = sortrows(world_coordinate_inter_adjustedFrameID',1);
world_coordinate_inter_adjustedFrameID = world_coordinate_inter_adjustedFrameID';


[Y_adjustedFrameID_inter,PSY_adjustedFrameID_inter]=mapminmax(world_coordinate_inter_adjustedFrameID(3,:),-1,1);
[X_adjustedFrameID_inter,PSX_adjustedFrameID_inter]=mapminmax(world_coordinate_inter_adjustedFrameID(4,:),-1,1);

world_coordinate_inter_normalized_adjustedFrameID=[world_coordinate_inter_adjustedFrameID(1,:); world_coordinate_inter_adjustedFrameID(2,:);Y_adjustedFrameID_inter;X_adjustedFrameID_inter];

other=[PSY; PSX; PSY_inter;PSX_inter;PSY_adjustedFrameID;PSX_adjustedFrameID;PSY_adjustedFrameID_inter;PSX_adjustedFrameID_inter];

csvwrite('pixel_coordinate_inter.csv', pixel_coordinate_inter);

csvwrite('pixel_coordinate_inter_normalized.csv', pixel_coordinate_inter_normalized);

csvwrite('pixel_coordinate_inter_normalized01.csv', pixel_coordinate_inter_normalized01);

csvwrite('world_coordinate.csv', world_coordinate);

csvwrite('world_coordinate_normalized.csv', world_coordinate_normalized);

csvwrite('world_coordinate_inter.csv', world_coordinate_inter);

csvwrite('world_coordinate_inter_normalized.csv', world_coordinate_inter_normalized);

csvwrite('world_coordinate_inter_normalized01.csv', world_coordinate_inter_normalized01);

csvwrite('world_coordinate_adjustedFrameID.csv', world_coordinate_adjustedFrameID);

csvwrite('world_coordinate_normalized_adjustedFrameID.csv', world_coordinate_normalized_adjustedFrameID);

csvwrite('world_coordinate_inter_adjustedFrameID.csv', world_coordinate_inter_adjustedFrameID);

csvwrite('world_coordinate_inter_normalized_adjustedFrameID.csv', world_coordinate_inter_normalized_adjustedFrameID);

save('other.mat', 'other');

