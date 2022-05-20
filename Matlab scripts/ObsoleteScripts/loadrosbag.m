clear all
close all

rosbag info corner_event.bag

bag = rosbag('corner_event.bag');

bsel = select(bag,'Topic','/dvs/events');



% load('xstore.mat');
% load('ystore.mat');
% load('times.csv');
% msg = readMessages(bsel);
%% 
for row=1:bsel.NumMessages
   pc(row) = readMessages(bsel,row);
end
x = zeros();
% xstore = zeros(593,1);
%%
xstore = cell(593,1);
ystore = cell(593,1);
tstore = cell(593,1);
time_stamp = struct('Ns',0,'s',0);
for v=1:593
   
    s = size(pc{1,v}.Events,1);
    x = zeros(s,1);
    y = zeros(s,1);
    t = cell(s,1);
%     tn = zeros(s,1);
    
    for i=1:s
        x(i,1) = pc{1,v}.Events(i,1).X;
        y(i,1) = pc{1,v}.Events(i,1).Y;
        timeStamp.s = (pc{1,v}.Events(i,1).Ts.Sec); 
        timeStamp.Ns = pc{1,v}.Events(i,1).Ts.Nsec;
        t{i,1} = timeStamp;
    end
        xstore{v,1} = x;
        ystore{v,1} = y;
        tstore{v,1} = t;
end
xyt = [xstore ystore tstore];

%% 

tsel = select(bag,'Topic','/dvs/events');
tmsg = cell(1,tsel.NumMessages);
for i=1:tsel.NumMessages
    tmsg(i) = readMessages(tsel,i);
end


% t1 = tmsg{1,1}.Events(1,1).Ts.seconds + (tmsg{1,1}.Events(1,1).Ts.Nsec)*10^-9;

timeOfFirstEvent = zeros(593,1);
for i=1:593
    timeOfFirstEvent(i,1) = tmsg{1,i}.Events(1,1).Ts.seconds + (tmsg{1,1}.Events(1,1).Ts.Nsec)*10^-9;
end    



