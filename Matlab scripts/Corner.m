classdef Corner < handle
    % Corner Class to contain all associated data and functions for a corner feature
 

    properties (Access = private)
        
        x;                  %initial x measurment: Found by manually going through Events to find on closest to harris corner detection
        y;                  %initial y measurment
        t1;
        xd;                 %initial velocity measurment not really relevant anyways
        yd;                 %initial y velocity measurment
        acc;                %'acceleration'

        dt;                 %time step
        F;                  %process matrix
        W;                  
        Q;                  %Process covariance?
        H;                  %measurment matrix
        E;                  %covariance
        Xp;                 %x_hat/KF x   
        R;                  %measurment covariance matrix

        skip;               %skip for when no measurment is found
        size_t1;            %size of previous possible features
        iter;               %count for message number/loop number
        num_corners;        %number of corners input
        corner_relat;       %corner position relations to each other

        %plots 
        el;                 %ellipse plot
        g;                  %feature selected as measurment
        cr;                 %KF prediction position
             
    end
    properties (Access = public)
        KF_feature;             %stores points calculated by KF
        avgs;                   %average position out of those found within error ellipse
        cmd_Xp;                 %prediction matrix containing predicted positions and velocities
        num_skips;              %number of skips 

    end 

    methods
        %% Initialise matrices and constants
        function self = Corner(KFvariables)
            % Construct an instance of this class
            self.num_corners = size(KFvariables,2);
            self.num_skips = zeros(self.num_corners,1);
            self.avgs = cell(self.num_corners,1);
            self.corner_relat = zeros(self.num_corners-1,2,self.num_corners);
            %Loop here for each corner being 'tracked'
            for C_idx = 1:size(KFvariables,2)
                xd_ = KFvariables{C_idx}{1};                %velocity
                yd_ = KFvariables{C_idx}{2};                %velocity
                x_ = KFvariables{C_idx}{3};                 %initial x state. Taken from harris corner detection on octogon shape 
                y_ = KFvariables{C_idx}{4};                 %initial y state 
    
    
                self.x(C_idx) = KFvariables{C_idx}{5};            %initial x measurment: Found by manually going through Events to find on closest to harris corner detection
                self.y(C_idx) = KFvariables{C_idx}{6};            %initial y measurment
                self.t1(C_idx) = KFvariables{C_idx}{7};
                self.xd(C_idx) = KFvariables{C_idx}{8};           %initial velocity measurment not really relevant anyways
                self.yd(C_idx) = KFvariables{C_idx}{9};           %initial y velocity measurment
                self.acc(C_idx) = KFvariables{C_idx}{10};         %'acceleration'
                        
                w = KFvariables{C_idx}{11};                       %process noise 
                vx = KFvariables{C_idx}{12};                      %x measurment noise:  stddv of the events were 4-5 pixels
                vy = KFvariables{C_idx}{13};                      %y measurment noise 
    
                num_messages = KFvariables{C_idx}{14};
                
                %Initialise storing matrices
                self.KF_feature(:,:,C_idx) = zeros(num_messages,2);
                self.cmd_Xp = cell(1,6);
                self.dt = KFvariables{C_idx}{15};           %time steps   
                
                X_ = [x_;                                   %initial state matrix
                       y_;
                       xd_;
                       yd_];
                self.Xp{C_idx} = X_;                        %Initialise first KF reading         
                                   
                self.R{C_idx} = [vx^2 0;                    %measurment covariance matrix
                                 0 vy^2];
            end                                         %End loop here, the following variables are same for each corner since they belong to the same system/object
            num_messages = KFvariables{1}{14};
            self.F = cell(num_messages,1);              %Initialise F,W,Q matrices
            self.W = cell(num_messages,1);
            self.Q = cell(num_messages,1);
            %Loop to populate F,W,Q matrices since dt changes slighty between
            %each message
            for j=1:num_messages
                 self.F{j} = [1 0 self.dt(j) 0;      %state update matrice
                            0 1 0 self.dt(j);
                            0 0 1 0;
                            0 0 0 1];
                         
                self.W{j} = [1/2*self.dt(j)^2;    %
                            1/2*self.dt(j)^2;
                            self.dt(j);
                            self.dt(j)];
    
                 self.Q{j} = [self.dt(j)^4 0 self.dt(j)^3 0;        %covariance
                            0 self.dt(j)^4 0 self.dt(j)^3;
                            self.dt(j)^3 0 self.dt(j)^2 0;
                            0 self.dt(j)^3 0 self.dt(j)^2].*w^2;
            end  
            self.H = [1 0 0 0;                                      %measurment matrix 
                      0 1 0 0];
    
            for l = 1:self.num_corners
                self.E{l} = self.Q{1};                         %set initial variance
            end
            self.size_t1(C_idx) = 100000;       
            self.iter = 1;  
            for i=1:self.num_corners                           %Obtain positional relations between each corner from each other corner
                count = 1;
                for j=1:self.num_corners
                    if i ~= j                               %check to exclude a corner checking relation to itself
                        self.corner_relat(count,1,i) = self.x(i) - self.x(j);
                        self.corner_relat(count,2,i) = self.y(i) - self.y(j);
                        count = count + 1;
                    end 
                end
            end
        end

%% KF Loop
        function KF_loop(self,xyt)
           for C_idx = 1:self.num_corners
                %initial measurment
                Xm = [self.x(C_idx);
                    self.y(C_idx);
                    self.xd(C_idx);
                    self.yd(C_idx);];                  
                self.skip = false;
                try delete(self.g); end;
                if self.iter > 1
                    %Get ellipse bounds
                    [a,b,h,k,ell] = error_ellipse([self.KF_feature(self.iter-1,1,C_idx) self.KF_feature(self.iter-1,2,C_idx)],self.E{C_idx}(1:2,1:2));     %get ellipse bounds
                    try delete(self.el); end;
                    self.el = plot(ell(:,1) + h,ell(:,2) + k,'-');         %Plot ellipse around KF point
                    %Get features that are inside the ellipse, with the
                    %most likely one placed at the top of the list
                    t2 = self.getPointsInEllisp(xyt(1,:),[self.KF_feature(self.iter-1,1,C_idx) self.KF_feature(self.iter-1,2,C_idx)],a,b,self.t1(C_idx),C_idx);    
                    size_t2 = size(t2,1);
                    %If t2 returns and is NOT empty
                    if size_t2 ~= 0         
                        Xm(1) = t2(1,1);                                   %Extract value from t2 and set as measurment                 
                        Xm(2) = t2(1,2);
                        self.t1(C_idx) = xyt{1,3}{t2(1,5)}.Ns*10^-9 + xyt{1,3}{t2(1,5)}.s; %Update last event time
                        self.g = plot(Xm(1),Xm(2),'*','color','g');                    %Plot the t2 point as green *
                    %If t2 returns and IS empty, make ellipse a little
                    %bigger to see if there are any features just out of
                    %reach and check again
                    else
                        [a,b,h,k,ell] = error_ellipse([self.KF_feature(self.iter-1,1,C_idx) self.KF_feature(self.iter-1,2,C_idx)],1.5*self.E{C_idx}(1:2,1:2));     %get ellipse bounds
                        try delete(self.el); end;
                        self.el = plot(ell(:,1) + h,ell(:,2) + k,'-');
                        [t2,p2] = self.getPointsInEllisp(xyt(1,:),[self.KF_feature(self.iter-1,1,C_idx) self.KF_feature(self.iter-1,2,C_idx)],a,b,self.t1(C_idx),C_idx);    
                        size_t2 = size(t2,1);
                        %If t2 is no longer 0
                        if size_t2 ~= 0
                            Xm(1) = t2(1,1);                 
                            Xm(2) = t2(1,2);
                            self.t1(C_idx) = xyt{1,3}{t2(1,5)}.Ns*10^-9 + xyt{1,3}{t2(1,5)}.s;
                            self.g = plot(Xm(1),Xm(2),'*','color','g');
                        %If t2 is STILL empty, take the closest feature event to
                        %the predicted point as the measurment and set skip
                        %to true 
                        else 
                            [idx,D] = knnsearch([xyt{1,1} xyt{1,2}],[p2(1) p2(2)]);
                            Xm(1) = p2(1);    
                            Xm(2) = p2(2);    
                            self.skip = true;
                            self.g = plot(Xm(1),Xm(2),'*','color','g');
                            self.t1(C_idx) = xyt{1,3}{idx}.Ns*10^-9 + xyt{1,3}{idx}.s;       
                        end   
                    end 
                    self.size_t1(C_idx) = size_t2;                         %update size of points in ellipse
                end 
                self.KFupdate(Xm,C_idx);                                   %update step
           end 
            self.iter = self.iter + 1;
        end 
 %%
        function KFupdate(self,Xm,C_idx)
            avg_acc = self.acc(1);
            %Update Xp
            self.Xp{C_idx} = self.F{self.iter} * self.Xp{C_idx} + self.W{self.iter} .* avg_acc;
            %Update E
            self.E{C_idx} = self.F{self.iter} * self.E{C_idx} * self.F{self.iter}' + self.Q{self.iter};                 %Predict next covariance
            %Calculate K
            K = self.E{C_idx}*self.H'*inv(self.H*self.E{C_idx}*self.H'+[500 0;0 500]);          %If it is determined that there were no events, i.e. no measurments set noise really high so it the Kalman gain ignores it
            self.num_skips(C_idx) =  self.num_skips(C_idx) + 1;
            %If no measurments were taken earlier, leave K as above, in
            %order to ignore the measurment component, otherwise override
            %the above line with the line below
            if self.skip == false  
                K = self.E{C_idx}*self.H'*inv(self.H*self.E{C_idx}*self.H'+self.R{C_idx});           %Kalman Gain     K = E*H'*inv(H*E*H'+R);
                self.num_skips(C_idx) =  self.num_skips(C_idx) - 1;
            end 
            %Predict next Xp
            self.Xp{C_idx} = self.Xp{C_idx} + K*(Xm(1:2,:) - self.H*self.Xp{C_idx});                     %update state estimate
            
            self.cmd_Xp{C_idx}(:,:,self.iter) = self.Xp{C_idx};
            cdmXp = self.Xp{C_idx}                                                              %print to console                         
            self.KF_feature(self.iter,:,C_idx) = [self.Xp{C_idx}(1) self.Xp{C_idx}(2)];         %extract x,y pos
            %Predict next E
            self.E{C_idx} = (eye(4) - K*self.H)*self.E{C_idx};                                    %update covariance estimate
%             try delete(self.cr); end;
            self.cr = scatter(self.KF_feature(self.iter,1,C_idx),self.KF_feature(self.iter,2,C_idx),'+','k');
        end
    end 

    methods (Access = private)
%% Look for features within the elipse bounds & determine which to choose if any
    function [features,P] = getPointsInEllisp(self,curr_features,KF_feature,a,b,t1,C_idx)
        % Gets the events from the next message which fall within the
        % bounds of the ellipse
        features = zeros(10,5);
        counter = 1;
        %Search through all events and select those that fall within
        %the bounds of the ellipse and within 0.05 seconds of the last one
        for j=1:size(curr_features{1},1)
            if (curr_features{1,1}(j)-KF_feature(1))^2/b^2 + (curr_features{1,2}(j)-KF_feature(2))^2/a^2 < 1 %&&  0.055>((next_features{1,3}{j}.Ns*10^-9 + next_features{1,3}{j}.s) - t1)  %(x-x0)^2 + (y-y0)^2 < r^2, the point (x,y) is inside the circle,
                if (curr_features{1,3}{j}.Ns*10^-9 + curr_features{1,3}{j}.s) - t1 < 0.05
                    features(counter,1) = curr_features{1,1}(j);    %x coordinate
                    features(counter,2) = curr_features{1,2}(j);    %y coordinate
                    features(counter,3) = pdist([KF_feature(1,1),KF_feature(1,2);curr_features{1,1}(j),curr_features{1,2}(j)]); %distance of the feature to the KF point
                    features(counter,4) = (curr_features{1,3}{j}.Ns*10^-9 + curr_features{1,3}{j}.s) - t1;  %the time between this feature and the last
                    features(counter,5) = j;                                                                %the features index in the ROS message
                    counter = counter + 1;
                end
            end 
        end
        features = features(any(features,2),:);                            %remove any zeros
        features = sortrows(features,[4 3]);                               %sort by time
        [~,uidx] = unique(features(:,[1 2]),'rows','stable');              %remove any double events at same pixel, keep smallest t diff as sorted above
        features = features(uidx,:);
        switch C_idx                                                        
            case 1   %for corner 1
                avgV = zeros(1,2);
                prev_set = zeros(self.num_corners-1,2);
                count = 1;
                %collect the velocities of corners2-6 from previous step
                for i=1:self.num_corners
                    if i ~= 1   %excluding the current corner in question
                        prev_set(count,:) = self.KF_feature(self.iter-1,:,i);
                        avgV(count,:) = self.Xp{count}(3:4,1)';
                        count = count + 1;
                    end 
                end
                avgV = rmoutliers(avgV);    %remove possible outliers
                avgV = mean(avgV);          %average
                p = zeros(size(prev_set,1),2);
                %using corners 2-6, obtain where each corner believed
                %corner 1 should have been 
                for i=1:size(prev_set,1)
                        p(i,:) = self.corner_relat(i,:,1) + prev_set(i,:);
                end
                p = mean(p);    %get the average 
                p2 = (self.dt(self.iter)*avgV) + p; %extropolate from that point using the average velocity calculated above and the supposed position
                P = p2;
                if (p2(1)-KF_feature(1))^2/b^2 + (p2(2)-KF_feature(2))^2/a^2 < 1    %check if p2 is within ellipse
                   [idx,D] = knnsearch(features(:,1:2),[p2(1) p2(2)]);              %find closest feature to p2,from those features that fit the criteria
                   D = pdist([features(idx,1:2);KF_feature]);              %distance from that feature and the ellipse centre
                   r = min([a b]);                                         %get the smallest ellipse radius
                    if D > r                                               %if D is larger than  the minor radius
                        features = features(idx,:);                        %take it as measurment but set skip to true
                        self.skip = true;
                    else
                        features = features(idx,:);                        %if D is smaller, select it as the measrument
                    end 
                else    %if p2 is outside of the ellipse find the closest point outside the ellipse to p2 and set skip to true
                    [idx,D] = knnsearch([curr_features{:,1} curr_features{:,2}],[p2(1) p2(2)]);
                    features(1,1) = curr_features{1,1}(idx);    
                    features(1,2) = curr_features{1,2}(idx);    
                    features(1,3) = pdist([KF_feature(1,1),KF_feature(1,2);curr_features{1,1}(idx),curr_features{1,2}(idx)]);
                    features(1,4) = (curr_features{1,3}{idx}.Ns*10^-9 + curr_features{1,3}{idx}.s) - t1;
                    features(1,5) = idx;
                    self.skip = true;
                end
            otherwise   %for all other corners
                curr_point = self.KF_feature(self.iter,:,1);
                p = self.corner_relat(1,:,C_idx) + curr_point; %get the relation of the current point to the predicted point for corner 1
                P = p;                                         %based off of corner 1's prediction, predict where this corner would be
                if (p(1)-KF_feature(1))^2/b^2 + (p(2)-KF_feature(2))^2/a^2 < 1  %if the prediction is within the ellipse
                   [idx,D] = knnsearch(features(:,1:2),[p(1) p(2)]);            %find closest feature that fit criteria to prediction and its distance to it
                    r = min([a b]);                                             %minor radius
                    if D > r                                                    %if too far, take it as measurment but set skip to true
                        features = features(idx,:);
                        self.skip = true;
                    else
                        features = features(idx,:);                             %else take it as measurment
                    end 
                else    %same as for corner 1
                    [idx,D] = knnsearch([curr_features{:,1} curr_features{:,2}],[p(1) p(2)]);
                    features(1,1) = p(1);    
                    features(1,2) = p(2);    
                    features(1,3) = pdist([KF_feature(1,1),KF_feature(1,2);curr_features{1,1}(idx),curr_features{1,2}(idx)]);
                    features(1,4) = (curr_features{1,3}{idx}.Ns*10^-9 + curr_features{1,3}{idx}.s) - t1;
                    features(1,5) = idx;
                    self.skip = true;
                end
        end        
    end 
    end 
end 




