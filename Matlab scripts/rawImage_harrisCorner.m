clear all
close all
%%
rosbag info corner_event.bag

bag = rosbag('corner_event.bag');

bsel = select(bag,'Topic','/dvs/image_raw');
% sel = select(bag,'Topic','/feature_events');

msg = readMessages(bsel);

img = readImage(msg{43});

corners = detectHarrisFeatures(img);
imshow(img);
hold on;
plot(corners.selectStrongest(50));


%% 
c = cell(447,1);

for i=88:447
   img = readImage(msg{i});
   corners = detectHarrisFeatures(img);
   imshow(img);
   hold on;
   plot(corners.selectStrongest(50));
   count = 1;
   for j=1:size(corners,1)
       if corners.Location(j,2) < (130+(i-87)*1)  && corners.Location(j,2) > (46+(i-87)*1) && corners.Location(j,1) < 220 && corners.Location(j,1) > 160
            c{i}(count,:) = corners.Location(j,:);
            count = count + 1;
       end      
   end       
end
%% To compare second data set uncomment here
% load('HC43to75.mat');
% a = c;
% a = a(~cellfun('isempty',a)); %remove zeros
% 
% load('HC76to88.mat');
% b = c;
% b = b(~cellfun('isempty',b)); %remove zeros
% 
% load('HC88to118.mat');
% e = c;
% e = e(~cellfun('isempty',e)); %remove zeros
% 
% abc = [a;b;e];
% for i=1:size(abc,1)
%     x = abc{i,1}(:,1);
%     y = abc{i,1}(:,2);
%     xc = mean(x);
%     yc = mean(y);
%     angles = mod((atan2d((y-yc),(x-xc))-200),360);
%     [sortedAngles, sortedIndexes] = sort(angles);
%     x = x(sortedIndexes);  % Reorder x and y with the new sort order.
%     y = y(sortedIndexes);
%     abc{i,1}(:,1) = x;
%     abc{i,1}(:,2) = y;
% end
% for i=1:size(abc,1)
% %     count = 1;
%     for j=1:6
%         HC_plot(j,:,i) = abc{i}(j,:);
% %         count = count + 1;
%     end
% end
% load('KF56to158.mat');
% KF_features = cell(593,1);
% for i = 1:593
%     count = 1;
%     for j=1:6
%        KF_features{i}(count,:) = KF56to158(i,:,j);
%        count = count + 1;
%    end 
% end
% 
% for i=1:99      %cut data off so that the HC and KF end at roughly the same point
% %     count = 1;
%     for j=1:6
%         KF_plot(j,:,i) = KF_features{i}(j,:);
% %         count = count + 1;
%     end
% end

%%
close all
% clear all
load('compare.mat');                                
figure (1)
xyHC_KF = cell(2,6);
x11 = double(squeeze(HC_plot(1,1,:)));
% x11(49) = x11(49) + 0.000001;             %uncomment for data set2                                      
y11 = double(squeeze(HC_plot(1,2,:)));
% y11(49) = y11(49) + 0.000001;             %uncomment for data set2 
x21 = double(squeeze(KF_plot(1,1,:)));
y21 = double(squeeze(KF_plot(1,2,:)));
xyHC_KF{1,1} = [x11 y11];
xyHC_KF{2,1} = [x21 y21];
subplot(3,2,1)
scatter(x11,y11,'b');
hold on
scatter(x21,y21,'r');
x12 = double(squeeze(HC_plot(2,1,:)));
y12 = double(squeeze(HC_plot(2,2,:)));
x22 = double(squeeze(KF_plot(2,1,:)));
y22 = double(squeeze(KF_plot(2,2,:)));
xyHC_KF{1,2} = [x12 y12];
xyHC_KF{2,2} = [x22 y22];
subplot(3,2,2)
scatter(x12,y12,'b');
hold on
scatter(x22,y22,'r');
x13 = double(squeeze(HC_plot(3,1,:)));
y13 = double(squeeze(HC_plot(3,2,:)));
x23 = double(squeeze(KF_plot(3,1,:)));
y23 = double(squeeze(KF_plot(3,2,:)));
xyHC_KF{1,3} = [x13 y13];
xyHC_KF{2,3} = [x23 y23];
subplot(3,2,3)
scatter(x13,y13,'b');
hold on
scatter(x23,y23,'r');
x14 = double(squeeze(HC_plot(4,1,:)));
y14 = double(squeeze(HC_plot(4,2,:)));
x24 = double(squeeze(KF_plot(4,1,:)));
y24 = double(squeeze(KF_plot(4,2,:)));
xyHC_KF{1,4} = [x14 y14];
xyHC_KF{2,4} = [x24 y24];
subplot(3,2,4)
scatter(x14,y14,'b');
hold on
scatter(x24,y24,'r');
x15 = double(squeeze(HC_plot(5,1,:)));
y15 = double(squeeze(HC_plot(5,2,:)));
x25 = double(squeeze(KF_plot(5,1,:)));
y25 = double(squeeze(KF_plot(5,2,:)));
xyHC_KF{1,5} = [x15 y15];
xyHC_KF{2,5} = [x25 y25];
subplot(3,2,5)
scatter(x15,y15,'b');
hold on
scatter(x25,y25,'r');
x16 = double(squeeze(HC_plot(6,1,:)));
y16 = double(squeeze(HC_plot(6,2,:)));
x26 = double(squeeze(KF_plot(6,1,:)));
y26 = double(squeeze(KF_plot(6,2,:)));
xyHC_KF{1,6} = [x16 y16];
xyHC_KF{2,6} = [x26 y26];
subplot(3,2,6)
scatter(x16,y16,'b');
hold on
scatter(x26,y26,'r');
hold off;

subplot(3,2,1)
title('Corner 1');
set(gca,'YDir','reverse');
xlim([70 240]);
ylim([60 180]);
legend('Harris Corner','Kalman Filter','Location','southwest')

subplot(3,2,2)
title('Corner 2');
set(gca,'YDir','reverse');
xlim([70 240]);
ylim([60 180]);

subplot(3,2,3)
title('Corner 3');
set(gca,'YDir','reverse');
xlim([70 240]);
ylim([60 180]);

subplot(3,2,4)
title('Corner 4');
set(gca,'YDir','reverse');
xlim([70 240]);
ylim([60 180]);

subplot(3,2,5)
title('Corner 5');
set(gca,'YDir','reverse');
xlim([70 240]);
ylim([60 180]);

subplot(3,2,6);
title('Corner 6');
set(gca,'YDir','reverse')
xlim([70 240]);
ylim([60 180]);
sgtitle('Kalman Filter vs Harris Corners for each corner of shape')

figure (2)

% Get the length of the first set of data.
length1 = length(x11);
% Get the length of the second set of data.
length2 = length(x21);
% Merge the two x axes.
x = sort([x11', x21'], 'ascend');
% Get new y1 values by interpolating the y value
% at the newly added x2 coordinates
y1New = interp1(x11, y11, x);
% Get new y2 values by interpolating the y value
% at the newly added x1 coordinates
y2New = interp1(x21, y21, x);
% Plot things:
subplot(2,1,1);
title('Raw data');
plot(x11, y11, 'bo','LineWidth', 2);
hold on;
plot(x21, y21, 'rd','LineWidth', 2);
% legend('HC', 'Interpolated HC');
axis([0 240 0 180]);
set(gca,'YDir','reverse')
legend('Harris Corner','Kalman Filter','Location','southwest')

subplot(2,1,2)
title('Interpolated Results');
scatter(x, y1New, 'b*', 'LineWidth', 2);
hold on
scatter(x, y2New, 'r*', 'LineWidth', 2);
% legend('KF', 'Interpolated KF');
grid on;
set(gca,'YDir','reverse')
xlim([0 240]);
ylim([0 180]);
legend('Harris Corner','Kalman Filter','Location','southwest')

subplot(2,1,2)
title('Interpolated Results');

subplot(2,1,1);
title('Raw data');

tHC = 0:1/25:1.04;                       
tKF = 0:0.03:1.05;                 
%for data set2 0:1/25:3.08;
%for data set2 0:0.03:2.94;
interped = cell(2,6);

for i=1:6
    % Merge the two x axes.
    x = sort([tHC, tKF], 'ascend');
    % Get new y1 values by interpolating the y value
    % at the newly added x2 coordinates
    x1New = interp1(tHC, xyHC_KF{1,i}(:,1), x);
    y1New = interp1(tHC, xyHC_KF{1,i}(:,2), x);
    % Get new y2 values by interpolating the y value
    % at the newly added x1 coordinates
    x2New = interp1(tKF, xyHC_KF{2,i}(:,1), x);
    y2New = interp1(tKF, xyHC_KF{2,i}(:,2), x);
    interped{1,i} = [x' x1New' x2New'];
    interped{2,i} = [x' y1New' y2New'];
    interped{1,i} = unique(interped{1,i},'rows');
    interped{2,i} = unique(interped{2,i},'rows');

end
    figure (3)
    scatter(interped{1,1}(:,2),interped{2,1}(:,2),'*','g');
    hold on
    scatter(interped{1,1}(:,3),interped{2,1}(:,3),'d','r');
    set(gca,'YDir','reverse')
    xlim([0 240]);
    ylim([0 180]);


acc = zeros(53,2,6);
for i=1:6
    for j=1:(size(interped{1,i},1)-1)
        xHC = interped{1,i}(j,2);
        yHC = interped{2,i}(j,2);
        xKF = interped{1,i}(j,3);
        yKF = interped{2,i}(j,3);
        
        xerrorR = (abs((xHC - xKF))/xHC)*100;
        xacc = 100 - xerrorR;
        
        yerrorR = (abs((yHC - yKF))/yHC)*100;
        yacc = 100 - yerrorR;
        if ~isnan(xacc) && ~isnan(yacc) 
            acc(j,1,i) = xacc;
            acc(j,2,i) = yacc;
        end
    end
        
%         avg_acc(:,i) = avg_acc(:,i)/(size(interped{1,i},1)-1);   
end
avg_acc = zeros(2,6);
min_acc = zeros(2,6);
max_acc = zeros(2,6);
med_acc = zeros(2,6);
std_acc = zeros(2,6);
num = size(acc,1);
for i=1:6
    avg_acc(:,i) = [mean(acc(:,1,i)); mean(acc(:,2,i))];
    min_acc(:,i) = [min(acc(:,1,i)); min(acc(:,2,i))];
    max_acc(:,i) = [max(acc(:,1,i)); max(acc(:,2,i))];
    med_acc(:,i) = [median(acc(:,1,i)); median(acc(:,2,i))];
    std_acc(:,i) = [std(acc(:,1,i)); std(acc(:,2,i))];
end

% figure (2)
% 
% % Get the length of the first set of data.
% length1 = length(x1);
% % Get the length of the second set of data.
% length2 = length(x2);
% % Merge the two x axes.
% x = sort([x1', x2'], 'ascend');
% % Get new y1 values by interpolating the y value
% % at the newly added x2 coordinates
% y1New = interp1(x1, y1, x);
% % Get new y2 values by interpolating the y value
% % at the newly added x1 coordinates
% y2New = interp1(x2, y2, x);
% % Plot things:
% subplot(2,1,1);
% plot(x1, y1, 'bo-','LineWidth', 2);
% hold on;
% plot(x2, y2, 'rd-','LineWidth', 2);
% % legend('HC', 'Interpolated HC');
% axis([0 240 0 180]);
% set(gca,'YDir','reverse')
% 
% subplot(2,1,2)
% scatter(x, y1New, 'b*', 'LineWidth', 2);
% hold on
% scatter(x, y2New, 'r*', 'LineWidth', 2);
% % legend('KF', 'Interpolated KF');
% grid on;
% set(gca,'YDir','reverse')
% xlim([0 240]);
% ylim([0 180]);
%%






