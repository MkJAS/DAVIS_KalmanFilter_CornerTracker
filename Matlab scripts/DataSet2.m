close all 
clear all

%%
load("Featuresxyt.mat");
load('times.csv');

%% Populate initial values
xd_ = -100;       %velocity
yd_ = 0;        %velocity
x_ = 203;        %initial x state. Taken from harris corner detection on octogon shape 
y_ = 82;        %initial y state 

x2_ = 215;
y2_ = 82;
x3_ = 220;
y3_ = 93;
x4_ = 216;
y4_ = 104;
x5_ = 203;
y5_ = 104;
x6_ = 198;
y6_ = 93;

 
idx = knnsearch([xyt{56,1} xyt{56,2}],[x_ y_]);
% idx_f = 123;
x = x_;                                        %initial x measurment: Found by manually going through Events to find on closest to harris corner detection
y = y_;                                        %initial y measurment
t1 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;
xd = -0.5;                                                   %initial velocity measurment not really relevant anyways
yd = 0.05;                                                  %initial y velocity measurment
acc = 50;                                                     %'acceleration'

 
idx = knnsearch([xyt{56,1} xyt{56,2}],[x2_ y2_]);
% x2 = (xyt{1,1}(idx));            
% y2 = (xyt{1,2}(idx));
x2 = x2_;
y2 = y2_;
t1_2 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;
x3 = x3_;            
y3 = y3_;
idx = knnsearch([xyt{1,1} xyt{1,2}],[x3_ y3_]);
% x3 = (xyt{1,1}(idx));            
% y3 = (xyt{1,2}(idx));
t1_3 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;
idx = knnsearch([xyt{1,1} xyt{1,2}],[x4_ y4_]);
% x4 = (xyt{1,1}(idx));            
% y4 = (xyt{1,2}(idx));
x4 = x4_;
y4 = y4_;
t1_4 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;
idx = knnsearch([xyt{1,1} xyt{1,2}],[x5_ y5_]);
% x5 = (xyt{1,1}(idx));            
% y5 = (xyt{1,2}(idx));
x5 = x5_;
y5 = y5_;
t1_5 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;
idx = knnsearch([xyt{1,1} xyt{1,2}],[x5_ y5_]);
% x5 = (xyt{1,1}(idx));            
% y5 = (xyt{1,2}(idx)); 
x6 = x6_;
y6 = y6_;
t1_6 = xyt{56,3}{idx}.Ns*10^-9 + xyt{56,3}{idx}.s;




w = 5000;                        %process noise 
vx = 3;                         %x measurment noise:  stddv of the events were 4-5 pixels
vy = 2;                         %y measurment noise 

num_messages = size(times,1);



dt = zeros(num_messages,1);

for i=1:592
    dt(i) = (xyt{i+1,3}{1,1}.Ns*10^-9 + xyt{i+1,3}{1,1}.s) - (xyt{i,3}{1,1}.Ns*10^-9 + xyt{i,3}{1,1}.s);
end
KFvariables1 = {xd_ yd_ x_ y_ x y t1 xd yd acc w vx vy num_messages dt};
% corner1 = Corner(KFvariables1);
KFvariables2 = {xd_ yd_ x2_ y2_ x2 y2 t1_2 xd yd acc w vx vy num_messages dt};
% corner2 = Corner(KFvariables2);
KFvariables3 = {xd_ yd_ x3_ y3_ x3 y3 t1_3 xd yd acc w vx vy num_messages dt};
% corner3 = Corner(KFvariables3);
KFvariables4 = {xd_ yd_ x4_ y4_ x4 y4 t1_4 xd yd acc w vx vy num_messages dt};
KFvariables5 = {xd_ yd_ x5_ y5_ x5 y5 t1_5 xd yd acc w vx vy num_messages dt};
KFvariables6 = {xd_ yd_ x6_ y6_ x6 y6 t1_6 xd yd acc w vx vy num_messages dt};
KFvariables = {KFvariables1 KFvariables2 KFvariables3 KFvariables4 KFvariables5 KFvariables6};
corners = Corner(KFvariables);


%% KF loop

figure (1)
axis([0 240 0 180]);
set(gca,'YDir','reverse')
hold on
%loop
 for i = 56:250
     try delete(Fevents); end;
     Fevents = scatter(xyt{i,1},xyt{i,2},'d','r');
     corners.KF_loop(xyt(i,:));
     
     pause(0.5);
 end

%%
KF_features = cell(593,1);
for i = 1:593
    count = 1;
    for j=1:6
       KF_features{i}(count,:) = a(i,:,j);
       count = count + 1;
   end 
end




