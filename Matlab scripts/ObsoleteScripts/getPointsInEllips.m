function [features] = getPointsInEllips(curr_features,KF_feature,a,b,t1)
% Gets the events from the next message which fall within the
% bounds of the ellipse
features = zeros(10,5);
counter = 1;
for j=1:size(curr_features{1},1)
            if (curr_features{1,1}(j)-KF_feature(1))^2/b^2 + (curr_features{1,2}(j)-KF_feature(2))^2/a^2 < 1 %&&  0.055>((next_features{1,3}{j}.Ns*10^-9 + next_features{1,3}{j}.s) - t1)  %(x-x0)^2 + (y-y0)^2 < r^2, the point (x,y) is inside the circle,
                features(counter,1) = curr_features{1,1}(j);
                features(counter,2) = curr_features{1,2}(j);
                features(counter,3) = pdist([KF_feature(1,1),KF_feature(1,2);curr_features{1,1}(j),curr_features{1,2}(j)]);
                features(counter,4) = (curr_features{1,3}{j}.Ns*10^-9 + curr_features{1,3}{j}.s) - t1;
                features(counter,5) = j;
                counter = counter + 1;
            end
end
features = features(any(features,2),:);                     %remove zeros

% if size(features,1) < 3
%     features = zeros(1,5);
%     features = features(any(features,2),:);
% 
% else
    features = sortrows(features,[3 4]);                        %sort by time
    [~,uidx] = unique(features(:,[1 2]),'rows','stable');       %remove any double events at same pixel, keep smallest t diff
    features = features(uidx,:);
% end
    
    if ~isempty(features)
        
        avg = [mean(features(:,1)) mean(features(:,2))];
        idx = knnsearch(features(:,1:2),[avg(1) avg(2)]);
        
        
        features = circshift(features,[(-idx+1) 0]);
 
        
    end
% temp = features;
% temp(:,4) = round(features(:,4),4);
% [temp, index] = sortrows(temp,[3 4]);
% features = features(index,:);



end

