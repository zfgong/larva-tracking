% recognize the light spot
% when the larva is cleared from the agar plate adjust the orientation of 
% infrared light source to get a brighter view and clear light spot

clear;
close all;

display('light spot region recognition!');

%managing directories for matlab execution and data saving
%workingdir='......\datademo\'  % current directory to run the program
outdir='M:\SOS\analyses\20160724\'; % set the destination directory to be the same as those online tracked data. one light spot recogonition for one day experiments.

lightregion={}; % this light region data is to be saved at each experiment

%vidobj=videoinput('winvideo',1,'YUY2_800x600');%get video input from camera %set the parameters as the same as in track.m
%preview(vidobj)
display('Write "return" when the light is in the arena');
keyboard;
filename='M:\SOS\video\20160724\light area.avi'
vidobj=VideoReader(filename);
firstbg=read(vidobj,1)% get snapshot 
%firstbg=ycbcr2rgb(firstbg);
goodbackground=firstbg; 

%set threshold to single out the light spot region
thresholdbackground=0.3                                                                                                                                                       ; % adjust threshold to extract visualize light spot % the threshold value can be adjustable

imspot=im2bw(firstbg,thresholdbackground);
% imspot=1-imspot; % If animal is white in a black background, comment this line
imspot=imclearborder(imspot); % Delete arena parts at the field of view border
% Find the biggest object and keep it as the only object
% Finding the biggest area may be unnecessary if the light spot if bright
[LO,num]=bwlabel(imspot,8);
Areas=regionprops(LO,'area');
area_val=[Areas.Area];
maxarea=max(area_val);
idxBig= find(maxarea == area_val);
it2=ismember(LO,idxBig); %ismember(A,B) returns an array containing 1 (true) where the data in A is found in B. Elsewhere, it returns 0 (false).
it2=imfill(it2,'holes');
[c r]=find(it2); % now the row and column of the nonzero light spot elements are indicated 

lightregion.image=it2;
% lightregion.region(:,1)=r;
% lightregion.region(:,2)=c;
lightregion.region = [r c]; % get the row and column value of points in light spot region
%save light spot, in both image and object indices
radiu = 10;
   
   N=2*radiu + 1;
    dilate_area=zeros(N,N);
    [i, j] = meshgrid(1:N, 1:N);
    idx = find((i-radiu-1).^2 + (j-radiu-1).^2 <= radiu^2);
    dilate_area(idx) = 1;
   
    lightregion.erodeimage = imdilate(lightregion.image, dilate_area);
    [Row, Col] = find(lightregion.erodeimage == 1);
    lightregion.eroderegion = [Col, Row];

cd(outdir);
save lightregion lightregion;
display('the image of lightregion');
figure,
subplot(1,3,1);imshow(firstbg)
subplot(1,3,2);imshow(it2)
subplot(1,3,3);imshow(lightregion.erodeimage)
% subplot(1,3,3);imshow(lightregion.erodeimage)
% subplot(1,3,3);imshow(lightregion.erodeimage)
