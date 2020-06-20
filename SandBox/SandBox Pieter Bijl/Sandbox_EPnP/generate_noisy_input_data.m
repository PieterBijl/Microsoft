function [A,point,Rt]=generate_noisy_input_data(n,std_noise,draw_plot)

% Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.       
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
% Francesc Moreno-Noguer, CVLab-EPFL, September 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 

if nargin<3 draw_plot=''; end
    
%true 3d position of the points--------------
xini=-2; xend=2;%bounds 
yini=-2; yend=2;
zini=4; zend=9;

%intrinsic camera parameters------------------
fx=354.4; %focal length in pixels
fy=354.4; 
f=3.9*10^-3; %50 mm of focal length

%m=fx/f
m=1; %f=1; %<<<<<<----------------------------
pixel_size = 1.1*10^-5;

u0=256; v0=256;
width=512; height=512;
A=[f 0 u0*pixel_size 0; 0 f v0*pixel_size 0; 0 0 1 0];
std_noise=std_noise*pixel_size;

%% Retrieve data from wireframe model
feature_points = 1/100*importdata('feature_points.txt'); %in meters

alpha=-170/(180/pi);
beta=30/(180/pi);
gamma=-80/(180/pi);
R = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
     sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
     -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma)];
i=1;
while i<=n
    point(i).Xcam=R*feature_points(:,i);
    point(i).Xcam(3) = point(i).Xcam(3) + 150;
    %project points into the image plane
    intermediate_points = R*feature_points(:,i);
    point(i).Ximg_true=project_3d_2d(A,point(i).Xcam);
    %point(i).Ximg_true= pixel_size*(intermediate_points(1:2)/0.2037+256);
    point(i).Ximg_true(3)=f;
   
    %coordinates in pixels
    point(i).Ximg_pix_true=point(i).Ximg_true(1:2)/pixel_size;    
    
   %check if the point is into the limits of the image
   uv=point(i).Ximg_pix_true;
   if uv(1)>0 & uv(1)<width & uv(2)>0 & uv(2)<height
       i=i+1;
   end
 %  fprintf('%.1f,%.1f,%d\n',uv(1),uv(2),i);
end

%add noise
for i=1:n
    noise=randn(1,2)*std_noise;
    point(i).Ximg(1,1)=point(i).Ximg_true(1)+noise(1);
    point(i).Ximg(2,1)=point(i).Ximg_true(2)+noise(2);
    point(i).Ximg(3,1)=f;
    
    %noisy coordinates in pixels
    point(i).Ximg_pix=point(i).Ximg(1:2)/pixel_size;
    
end
    
%the observed data will not be in the camera coordinate system. We put the center of the
%world system on the centroid of the data. We also assume that the data is rotated with
%respect to the camera coordenate system. 
centroid=[0 0 0]';
for i=1:n
    centroid=centroid+point(i).Xcam;    
end
centroid=centroid/n;



%x_rotation=random(0,45,1)*pi/180;
%y_rotation=random(0,45,1)*pi/180;
%z_rotation=random(0,45,1)*pi/180;
x_rotation = alpha;
y_rotation = beta;
z_rotation = gamma;
tx=0; ty=0; tz=150;
Rt=return_Rt_matrix(x_rotation,y_rotation,z_rotation,tx,ty,tz);
for i=1:n
   point(i).Xworld=feature_points(:,i); 
end


%plot noisy points in the image plane
if ~strcmp(draw_plot,'donotplot')
    figure; hold on;
    for i=1:n
        plot(point(i).Ximg_pix_true(1),point(i).Ximg_pix_true(2),'.','color',[1 0 0]);
        %txt=sprintf('%d',i);
        %text(point(i).Xcam(1),point(i).Xcam(2),point(i).Xcam(3),txt);
        plot(point(i).Ximg_pix(1),point(i).Ximg_pix(2),'o','color',[0 1 0],'markersize',5);
    end
    title('Noise in image plane','fontsize',20);
    grid on;
end






%draw 3d points
% figure; hold on;
% representation_offset=10;
% plot3(0,0,0,'.','color',[0 0 0],'markersize',20);
% for i=1:n
%    plot3(point(i).Xcam(1),point(i).Xcam(2),point(i).Xcam(3),'.','color',[1 0 0],'markersize',12);
%    txt=sprintf('%d',i);
%    text(point(i).Xcam(1),point(i).Xcam(2),point(i).Xcam(3),txt);
%    plot3(point(i).Xworld(1)+representation_offset,point(i).Xworld(2),point(i).Xworld(3),'.','color',[0.8 0.8 0],'markersize',12);
%    text(point(i).Xworld(1)+representation_offset,point(i).Xworld(2),point(i).Xworld(3),txt);   
%    plot3(point(i).Ximg(1),point(i).Ximg(2),point(i).Ximg(3),'.','color',[0 0 1],'markersize',12);
% end
% grid on;
% 