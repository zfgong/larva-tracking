% load 'D:\zhefeng\turn&explore\20161223zwq\160413 w1118ctr\motorData.mat'
% % load head_light_distance
% load 'D:\zhefeng\turn&explore\20161223zwq\160413 w1118ctr\lightregion.mat'
clear;
dir='D:\temp2\160413 w1118ctr';
cd(dir);
load motorData.mat
load lightregion.mat
contour = bwperim(lightregion.image);
wperim=bwperim(contour);
imcontour=wperim;
%
centerIndp=find(wperim>0);
[py,px]=ind2sub(size(wperim),centerIndp);
lightspot_contour.x=[px'];
lightspot_contour.y=[py'];
save lightspot_contour lightspot_contour

%%%%%% smooth the contour with sgolayfilt %%%%%%%%%%%%%%
boundaries = bwboundaries(lightregion.image);
numberOfBoundaries = size(boundaries, 1);

firstBoundary = boundaries{1};
% Get the x and y coordinates.
x = firstBoundary(:, 2);
y = firstBoundary(:, 1);
windowWidth = 799;
polynomialOrder = 12;
smoothX = sgolayfilt(x, polynomialOrder, windowWidth);
smoothY = sgolayfilt(y, polynomialOrder, windowWidth);

lightspot_contour_smooth.x=smoothX;
lightspot_contour_smooth.y=smoothY;
save lightspot_contour_smooth lightspot_contour_smooth
%%%% end edge smoothing and saving as lightspot_contour  %%%%%%%%

lightspot_contour.x=smoothX;
lightspot_contour.y=smoothY;

p = size(motorData,2);

for i = 1:p

    display(i);
    bodyomega = motorData{i}.bodyOmega;
    bodyomega = filter2([1], bodyomega);
    tailspeed = motorData{i}.tailSpeed;
    tailspeed = filter2([1], tailspeed);
    headtheta = motorData{i}.headTheta;
    headomega = motorData{i}.headOmega;
    headomega = filter2([1], headomega);
    cmspeed = motorData{i}.cmSpeed;
    headXY = motorData{i}.headXY;
%     headXYRaw = motorData{i}.headXYRaw;
    headXYrnd = motorData{i}.headXYrnd;
    midXY = motorData{i}.midXY;
    tailXY = motorData{i}.tailXY;
    bodyTheta = motorData{i}.bodyTheta;
    length = max(size(tailspeed));
    
    prop_turn=[];
    finish_turn=[];
    frame_turn_peaks=[];
    value_turn_peaks=[];
    frame_turn_peaks_aft_still=[];
    value_turn_peaks_aft_still=[];
    turn_still_starts=[];
    turn_still_ends=[];
    light_frame_turn_peaks=[];
    frame_turn_peaks_in_light=[];
    frame_turn_peaks_out_light=[];
    body_edge_angle=[];
    turn_avoid_light=[];
    turn_avoid_light_in_light=[];
    turn_avoid_light_out_light=[];
    body_edge_angle_in_light=[];
    body_edge_angle_out_light=[];
    still_starts_in_light=[];
    still_starts_out_light=[];
    distance_head_start=[];
    distance_head_finish=[];
    distance_head_start_in_light=[];
    distance_head_start_out_light=[];
    turn_headomega_peaks={};
    turn_headomega_peaks_out_light={};
    turn_headomega_peaks_in_light={};
    distance_headomega_zero = {};
    distance_headomega_zero_first=[];
     frame_headomega_zero ={};
     
    % judge bodyomega
    lambda = 0.53; % adjusting lambda from 0.8 to 0.4 improve sensitivity of bodyomega_peak detection to detect the flatter platform-like peak, 

    % find peaks of bodyomega
    frame_bodyomega_peak = cast_max_frames(bodyomega, lambda);
    frame_bodyomega_peak =frame_bodyomega_peak';
    value_bodyomega_peak = bodyomega(frame_bodyomega_peak);

    %judge the threshold of bodyomega depending on the amplitude of headtheta
    %peaks
    threshold_bodyomega = 0.3;
    set_threshold_bodyomega = 0.2; %threshold_imega when headtheta is large to exclude small passive turning
    frame_bodyomega_peak(find(abs(value_bodyomega_peak)<threshold_bodyomega)) = [];
    value_bodyomega_peak = bodyomega(frame_bodyomega_peak);

    %find peaks of headomega
    %exlude headomega peaks lower than 0.5
    lambda = 0.75; % adjusting lambda from 0.8 to 0.6 improve sensitivity of headomega_peak detection
    threshold_headomega = 0.35; % change 0.5 to 0.35 or any proper estimated threshold
    frame_headomega_peak = cast_max_frames(headomega, lambda);
    frame_headomega_peak = frame_headomega_peak';    
    value_headomega_peak = headomega(frame_headomega_peak);
    frame_headomega_peak(find(abs(value_headomega_peak)<threshold_headomega))=[];
    value_headomega_peak = headomega(frame_headomega_peak);
    % find tailspeed at zero less than a threshold
    threshold_tailspeed = 2.5;

%     % bad frames rejudgement tailspeed ==0 or bigger than 30 are removed
    bad_frames = find((tailspeed ==0)|(tailspeed >= 30));
    frame_headomega_peak(find(ismember(frame_headomega_peak,bad_frames)))=[];
    value_headomega_peak = headomega(frame_headomega_peak);
    frame_bodyomega_peak(find(ismember(frame_bodyomega_peak,bad_frames)))=[];
    value_bodyomega_peak = bodyomega(frame_bodyomega_peak);

    for d = 1: size(frame_headomega_peak,2)
        if ~isempty(find(ismember(frame_bodyomega_peak(:), (frame_headomega_peak(d)-10):(frame_headomega_peak(d)+10))))
            frame_headomega_peak(d)=-1;
            value_headomega_peak(d)=-100;
        end
    end
    frame_headomega_peak(find( frame_headomega_peak == -1))=[];
    value_headomega_peak(find( value_headomega_peak == -100))=[];
    
    
    tailspeed_zero = find(tailspeed <= threshold_tailspeed);
    tailspeed_nonzero = find(tailspeed > threshold_tailspeed);
    tailspeed_binary = tailspeed;
    tailspeed_binary(tailspeed_zero) = 0;
    tailspeed_binary(tailspeed_nonzero) =1;
    tailspeed_binary_diff= diff(tailspeed_binary);
    % change the frames before the first 1-0 drop to 1, just to exclude the possibility that the beginning constant zero masks all the following still_starts 
    tailspeed_binary_diff_neg = find(tailspeed_binary_diff==-1);
    tailspeed_binary_diff_neg_1 = tailspeed_binary_diff_neg(1);
    tailspeed_binary(1:tailspeed_binary_diff_neg_1)=1;
    tailspeed_binary_diff= diff(tailspeed_binary);
    %find the frames for the tailspeed change to be zero
    %change the short segment (<5, the threshold for short drop in locomotion) that is 0 between long string of 1 to 1,
    %that is, the short drop in tailspeed to 0 will be ignored
    %similarly, change the short segment (<100, the threshold for short rise in stop) that is 1 between long
    %strings of 0 to 1, that is, the short rise in tailspeed to 1 will be
    %ignored. but processing of short rise segment should be done before processing the short drop
    %segment
    tailspeed_binary_diff_ID = find(tailspeed_binary_diff~=0);
    tailspeed_binary_temp = tailspeed_binary;    
    a=4;
    for c=1:2
        for b=1:a
            for j=1:(size(tailspeed_binary_diff_ID,2)-1)
                if (tailspeed_binary_diff(tailspeed_binary_diff_ID(j))==1) && ((tailspeed_binary_diff_ID(j+1))-(tailspeed_binary_diff_ID(j))<=12.5*b)
                    tailspeed_binary(tailspeed_binary_diff_ID(j):tailspeed_binary_diff_ID(j+1))=0;                    
                end %change the short rise gap to 0 if moving period is shorter than 100 frames, this step must be done before changing short drop gap to 1 
            end
            
            for j=1:(size(tailspeed_binary_diff_ID,2)-1)
                if (tailspeed_binary_diff(tailspeed_binary_diff_ID(j))==-1) && ((tailspeed_binary_diff_ID(j+1))-(tailspeed_binary_diff_ID(j))<=(2.5*b))
                    tailspeed_binary(tailspeed_binary_diff_ID(j):tailspeed_binary_diff_ID(j+1))=1;
                end % change the short drop gap to 1 if stop period is shorter than 5 frames
            end
            tailspeed_binary_diff = diff(tailspeed_binary);
            tailspeed_binary_diff_ID = find(tailspeed_binary_diff~=0);
        end
    end
    still_starts = find(tailspeed_binary_diff ==-1);
    still_ends = find(tailspeed_binary_diff ==1);
    if ~isempty(still_ends)&&~isempty(still_starts)
        if still_ends(1)<still_starts(1)
            still_ends(1) =[];
        end
        if still_starts(end)>still_ends(end)
            still_starts(end)=[];
        end
    else
        still_starts =[];
        still_ends =[];
    end
    
    b = size(still_starts,2);
    for j=1:b
           
        % expand still periods by extending the still_ends later for 30
        % frames,
        still_ends(j)= still_ends(j)+30;
        
           frame_headomega_zero_1 =[];
           frame_headomega_zero_af_peak =[];
           distance_headomega_zero_array =[];
        if isempty(find(ismember(frame_headomega_peak, still_starts(j):still_ends(j))))    
            prop_turn(j)=0;
            turn_headomega_peaks{j}=[];
            frame_headomega_zero{j}=[];
            distance_headomega_zero{j}=[];
            distance_headomega_zero_first(j)=-1;
        else
            prop_turn(j)=size(frame_headomega_peak(find(ismember(frame_headomega_peak, still_starts(j):still_ends(j)))),2);
            turn_headomega_peaks{j}=value_headomega_peak(find(ismember(frame_headomega_peak, still_starts(j):still_ends(j))));
            %to get the first turn and related light avoidance
            value_headomega_peaks_temp = turn_headomega_peaks{j};
            frame_headomega_peaks_temp = find(ismember(headomega,value_headomega_peaks_temp));
            for m = 1: size(frame_headomega_peaks_temp,2)
                frame_headomega_zero_af_peak = -1;
                if m < size(frame_headomega_peaks_temp,2)
                    for n = frame_headomega_peaks_temp(m):frame_headomega_peaks_temp(m+1)
                        if headomega(n)*headomega(n+1) < 0
                            frame_headomega_zero_af_peak = find(headomega==headomega(n));
                            break;
                        end
                    end
                else
                    for n = frame_headomega_peaks_temp(m):length
                        if headomega(n)*headomega(n+1) < 0
                            frame_headomega_zero_af_peak = find(headomega==headomega(n));
                            break;
                        end
                    end
                end
                frame_headomega_zero_1 = [frame_headomega_zero_1 frame_headomega_zero_af_peak];
            end
            frame_headomega_zero{j}= frame_headomega_zero_1;
            
            
            % get distance value corresponding to frame_headomega_zero{j}
            
            distance_headomega_zero_array =[];
            for x = 1:size(frame_headomega_zero_1,2)
                if frame_headomega_zero_1(x) ~= -1
                    distance_headomega_zero_temp = min(sqrt((headXY(frame_headomega_zero_1(x),1)-lightspot_contour.x(:)).^2 +(headXY(frame_headomega_zero_1(x),2)-lightspot_contour.y(:)).^2));
                    if ~isempty(find(ismember(headXYrnd(frame_headomega_zero_1(x),:),lightregion.region,'rows')))
                        distance_headomega_zero_temp = -distance_headomega_zero_temp;
                    end
                else
                    distance_headomega_zero_temp = -1;   
                end
                distance_headomega_zero_array = [distance_headomega_zero_array distance_headomega_zero_temp];
            end
            distance_headomega_zero{j} = distance_headomega_zero_array;
            distance_headomega_zero_first(j) = distance_headomega_zero_array(1);
        end
        


                    
        % to calculate the angle between body axis (mid-tail) and edge of light
        % spot (nearest point to mid and tail on light spot contour)
        distance_mid_temp = sqrt((midXY(still_starts(j),1)-lightspot_contour.x(:)).^2 +(midXY(still_starts(j),2)-lightspot_contour.y(:)).^2);
        distance_mid = min(distance_mid_temp);
        lightspot_contour_mid = [lightspot_contour.x(find(distance_mid_temp ==distance_mid)) lightspot_contour.y(find(distance_mid_temp ==distance_mid))];
        distance_tail_temp = sqrt((tailXY(still_starts(j),1)-lightspot_contour.x(:)).^2 +(tailXY(still_starts(j),2)-lightspot_contour.y(:)).^2);
        distance_tail = min(distance_tail_temp);
        lightspot_contour_tail = [lightspot_contour.x(find(distance_tail_temp ==distance_tail)) lightspot_contour.y(find(distance_tail_temp ==distance_tail))];
        light_edge_ori = atan2((lightspot_contour_tail(2)-lightspot_contour_mid(2)), (lightspot_contour_tail(1)-lightspot_contour_mid(1))); 
        aat = unwrap(light_edge_ori)-unwrap(bodyTheta(still_starts(j)));
        body_edge_angle(j) = atan2(sin(aat), cos(aat));
        body_edge_angle_abs = abs(body_edge_angle(j));
        if body_edge_angle_abs>pi/2
            body_edge_angle_abs=pi-body_edge_angle_abs;
        end
        if body_edge_angle(j)<0
            body_edge_angle(j) = -body_edge_angle_abs;
        else
            body_edge_angle(j) = body_edge_angle_abs;
        end
        
        % judge if a turn result in close to light or further to light
        distance_head = min(sqrt((headXY(still_starts(j),1)-lightspot_contour.x(:)).^2 +(headXY(still_starts(j),2)-lightspot_contour.y(:)).^2));
        if ~isempty(find(ismember(headXYrnd(still_starts(j),:),lightregion.region,'rows')))    
            distance_head = -distance_head;
        end
        distance_head_start(j)=distance_head;
        % use still_ends i.e. the finish of stop stage, to judge the
        % headcast induced heading change
        distance_head = min(sqrt((headXY(still_ends(j),1)-lightspot_contour.x(:)).^2 +(headXY(still_ends(j),2)-lightspot_contour.y(:)).^2));       
        if ~isempty(find(ismember(headXYrnd(still_ends(j),:),lightregion.region,'rows')))
            distance_head = -distance_head;
        end
        distance_head_finish(j)=distance_head;
        
        if (distance_head_start(j) - distance_head_finish(j)) > 0
            turn_avoid_light(j) = 1; % 1 means failure in light avoidance
        else
            turn_avoid_light(j) = 0; % 0 means succuss in light avoidance
        end      
    end
    
    % save the data and parameters related to turning event
    turn_event{i}.prop_turn = prop_turn;
    turn_event{i}.turn_still_starts = still_starts;
    turn_event{i}.turn_still_ends = still_ends;
    turn_event{i}.distance_head_start = distance_head_start;
    turn_event{i}.distance_head_finish = distance_head_finish;
    turn_event{i}.turn_avoid_light = turn_avoid_light;
    turn_event{i}.body_edge_angle = body_edge_angle;
    turn_event{i}.turn_headomega_peaks = turn_headomega_peaks;
    turn_event{i}.threshold = threshold_bodyomega;
    turn_event{i}.length = length;
    turn_event{i}.frame_headomega_zero =frame_headomega_zero;
    turn_event{i}.distance_headomega_zero =distance_headomega_zero;
    turn_event{i}.distance_headomega_zero_first =distance_headomega_zero_first;


    
    % judge light related turning events
    distance_in_light_threshold = 50;
    distance_out_light_threshold = 20;
    
    light_frame_turn_peaks=[];
    frame_turn_peaks_in_light=[];
    frame_turn_peaks_out_light=[];
    turn_avoid_light_in_light=[];
    turn_avoid_light_out_light=[];
    body_edge_angle_in_light=[];
    body_edge_angle_out_light=[];
    still_starts_in_light=[];
    still_starts_out_light=[];
    
    headXY_temp = headXYrnd(still_starts,:);
    
    %%%% find turns in light  %%%%
    
    if ~isempty(find(ismember(headXY_temp,lightregion.region,'rows')))
        still_starts_in_light = still_starts(find(ismember(headXY_temp,lightregion.region,'rows')));

        %judge turn happening in light but within a distance
        % %         r = size(frame_turn_peaks_in_light,1);
        r = size(still_starts_in_light,2);
        %         display(still_starts_in_light);
        for j = 1:r
            distance_head_temp = min(sqrt((headXY(still_starts_in_light(j),1)-lightspot_contour.x(:)).^2 +(headXY(still_starts_in_light(j),2)-lightspot_contour.y(:)).^2));          
            % exclude turning out of the range
            if distance_head_temp > distance_in_light_threshold
                still_starts_in_light(j)=-1;
            end
            
            %exclude turning with the range to light contour but after deep entrance into light
            if distance_head_temp < distance_in_light_threshold
            % exclude turns after more than 5 seconds in light
                y = 0;
                for z = 1:still_starts_in_light(j)
                    if ~isempty(find(ismember(headXYrnd(z,:),lightregion.region,'rows')))
                        y=y+1;
                    end
                end
                
                if y > 300 
                    still_starts_in_light(j)=-1;
                end                
            end
        end
        still_starts_in_light(find(still_starts_in_light==-1))=[];       
    end
       display(still_starts_in_light);
       
    %%%% find turns out of light %%%%    
    if ~isempty(find(~ismember(headXY_temp,lightregion.region,'rows')))
        still_starts_out_light = still_starts(find(~ismember(headXY_temp,lightregion.region,'rows')));

        %judge turn happening out of light but within a distance
        q = size(still_starts_out_light,2); 
        for j= 1:q
            a = find(still_starts==still_starts_out_light(j));
            distance_head_temp = distance_head_start(a);            
            if distance_head_temp > distance_out_light_threshold
                still_starts_out_light(j)=-1;
            end
            
            %exclude turning within the range threshold to light contour but after staying long in light
            if distance_head_temp < distance_out_light_threshold               
                
                % exclude turns after more than 5 seconds in light
                y = 0;
                for z = 1:still_starts_out_light(j)
                    if ~isempty(find(ismember(headXYrnd(z,:),lightregion.region,'rows')))
                        y=y+1;
                    end
                end
                
                if y > 300
                    still_starts_out_light(j)=-1;
                end
            end            
        end
        still_starts_out_light(find(still_starts_out_light==-1))=[];
    end

    % merge all the turns in range of light response
    light_still_starts = still_starts(find(ismember(still_starts,still_starts_in_light) + ismember(still_starts,still_starts_out_light)));
    light_frame_turn_peaks_ID_pos = find(ismember(still_starts,light_still_starts));      
    light_turn_event{i}.turn_avoid_light = turn_avoid_light(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.body_edge_angle = body_edge_angle(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.distance_head_start = distance_head_start(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.distance_head_finish = distance_head_finish(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.turn_headomega_peaks = {turn_headomega_peaks{light_frame_turn_peaks_ID_pos}};  % {{}} is required 
    light_turn_event{i}.prop_turn = prop_turn(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.still_starts = still_starts(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.still_ends = still_ends(light_frame_turn_peaks_ID_pos);
    light_turn_event{i}.frame_headomega_zero = {frame_headomega_zero{light_frame_turn_peaks_ID_pos}};
    light_turn_event{i}.distance_headomega_zero = {distance_headomega_zero{light_frame_turn_peaks_ID_pos}};
    light_turn_event{i}.distance_headomega_zero_first = distance_headomega_zero_first(light_frame_turn_peaks_ID_pos);
    
end
cd(dir);
save turn_event turn_event
save light_turn_event light_turn_event

