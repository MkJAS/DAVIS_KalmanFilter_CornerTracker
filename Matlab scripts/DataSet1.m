close all 
clear all

%%
load("Featuresxyt.mat");    %.mat containing the extracting x,y values for all the events within the .bag file
load('times.csv');          %.csv containing the time of the first event, out of all the events not the feature extracted ones, for each message

%% Populate initial values

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%vvvvvvvvvvvvvvvvvvv Initial measurments vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
x_ = 95;        %initial x state. Taken from harris corner detection on octogon shape 
y_ = 78;        %initial y state 
x2_ = 110;
y2_ = 79;
x3_ = 115;
y3_ = 90;
x4_ = 108;
y4_ = 102;
x5_ = 93;
y5_ = 101;
x6_ = 88;
y6_ = 89;
%require only one set of velocities since corners belong to the same object
xd_ = 10;       %velocity - guessed
yd_ = 0;        %velocity - guessed
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%%%%%%%%%%%%%%%% Initial Measurments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%vvvvvvvvvvvvvvvvvvv Initial guesses vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
x = (xyt{1,1}(123));                                        %initial x measurment: Found by manually going through Events to find on closest to harris corner detection
y = (xyt{1,2}(123));                                        %initial y measurment
t1 = xyt{1,3}{123}.Ns*10^-9 + xyt{1,3}{123}.s;              %time of that measurment
 
% idx = knnsearch([xyt{1,1} xyt{1,2}],[x2_ y2_]);           %it is possible
% to use the knnsearch function to find the feature which is closest to the
% harris corner value instead of manually going through the messages. This
% would be the way to do this in the case of using a live data stream
% x2 = (xyt{1,1}(idx));            
% y2 = (xyt{1,2}(idx));
x2 = 110;                                               %remaining corner guesses were set to just be the same as the measurments
y2 = 79;
t1_2 = xyt{1,3}{147}.Ns*10^-9 + xyt{1,3}{147}.s;        %the times were taken from the closest feature event to the harriss corner point
x3 = 114;            
y3 = 90;
% idx = knnsearch([xyt{1,1} xyt{1,2}],[x3_ y3_]);
% x3 = (xyt{1,1}(idx));            
% y3 = (xyt{1,2}(idx));
t1_3 = xyt{1,3}{120}.Ns*10^-9 + xyt{1,3}{120}.s;
% idx = knnsearch([xyt{1,1} xyt{1,2}],[x4_ y4_]);
% x4 = (xyt{1,1}(idx));            
% y4 = (xyt{1,2}(idx));
x4 = 108;
y4 = 102;
t1_4 = xyt{1,3}{45}.Ns*10^-9 + xyt{1,3}{45}.s;
% idx = knnsearch([xyt{1,1} xyt{1,2}],[x5_ y5_]);
% x5 = (xyt{1,1}(idx));            
% y5 = (xyt{1,2}(idx));
x5 = 93;
y5 = 101;
t1_5 = xyt{1,3}{146}.Ns*10^-9 + xyt{1,3}{146}.s;
% idx = knnsearch([xyt{1,1} xyt{1,2}],[x5_ y5_]);
% x5 = (xyt{1,1}(idx));            
% y5 = (xyt{1,2}(idx)); 
x6 = 88;
y6 = 89;
t1_6 = xyt{1,3}{5}.Ns*10^-9 + xyt{1,3}{15}.s;

xd = 0.5;                                                   %initial velocity measurment not really relevant anyways
yd = 0.05;                                                  %initial y velocity measurment
acc = 50;                                                   %'acceleration'
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%%%%%%%%%%%%%%%% Initial Guesses %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


w = 5000;                       %process noise 
vx = 3;                         %x measurment noise:  stddv of the events were 4-5 pixels
vy = 2;                         %y measurment noise 

num_messages = size(times,1);
dt = zeros(num_messages,1);

for i=1:592             %collect the times between each message
    %time is stored as ROS time, which is a struct containing a seconds
    %amount and a nanoseconds amount
    dt(i) = (xyt{i+1,3}{1,1}.Ns*10^-9 + xyt{i+1,3}{1,1}.s) - (xyt{i,3}{1,1}.Ns*10^-9 + xyt{i,3}{1,1}.s);
end
%Take all the above variables for each corner and store them in separate cells
KFvariables1 = {xd_ yd_ x_ y_ x y t1 xd yd acc w vx vy num_messages dt};
KFvariables2 = {xd_ yd_ x2_ y2_ x2 y2 t1_2 xd yd acc w vx vy num_messages dt};
KFvariables3 = {xd_ yd_ x3_ y3_ x3 y3 t1_3 xd yd acc w vx vy num_messages dt};
KFvariables4 = {xd_ yd_ x4_ y4_ x4 y4 t1_4 xd yd acc w vx vy num_messages dt};
KFvariables5 = {xd_ yd_ x5_ y5_ x5 y5 t1_5 xd yd acc w vx vy num_messages dt};
KFvariables6 = {xd_ yd_ x6_ y6_ x6 y6 t1_6 xd yd acc w vx vy num_messages dt};

%store all the KF initial variables into 1 cell
KFvariables = {KFvariables1 KFvariables2 KFvariables3 KFvariables4 KFvariables5 KFvariables6};

%pass all corner KFvariables to the corner class
corners = Corner(KFvariables);


%% KF loop

figure (1)
axis([0 240 0 180]);
set(gca,'YDir','reverse')
hold on
%loop
 for i = 1:36
     try delete(Fevents); end;
     Fevents = scatter(xyt{i,1},xyt{i,2},'d','r');     %plot the current feature events
     corners.KF_loop(xyt(i,:));                        %pass the features to the KF
     pause(0.05);                                       %pause for plotting
 end
 results = corners.KF_feature;
%%
KF_features = cell(593,1);
for i = 1:593
    count = 1;
    for j=1:6
       KF_features{i}(count,:) = results(i,:,j);
       count = count + 1;
   end 
end

for i=1:36      %cut data off so that the HC and KF end at roughly the same point
%     count = 1;
    for j=1:6
        KF_plot(j,:,i) = KF_features{i}(j,:);
%         count = count + 1;
    end
end

