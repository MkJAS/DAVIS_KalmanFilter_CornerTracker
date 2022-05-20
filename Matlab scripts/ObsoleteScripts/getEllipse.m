function [a,b,h,k,r_ellipse] = getEllipse(E,V,corner)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    [vec,val] = eig(E);
    
    [largest_vecidx,r] = find(val == max(max(val)));
    largest_vec = vec(:,largest_vecidx);
    
    largest_val = max(max(val));

    if(largest_vecidx == 1)
        smallest_val = max(val(:,2))
        smallest_vec = vec(:,2)
    else
        smallest_val = max(val(:,1))
        smallest_vec = vec(1,:)
    end
    
%     evals = diag(val);
%     smalleig = inf;
%     
%     for i=1:length(evals)
%         if (evals(i) < smalleig)
%             smalleig = evals(i);
%             evec = vec(:, i);
%         end
%     end
%     
%     smallest_val = smalleig;
%     smallest_vec = evec;
    
    angle = atan2(largest_vec(2),largest_vec(1));
    if(angle < 0)
        angle = angle + 2*pi;
    end
    
    
    % Get the 95% confidence interval error ellipse
    chisquare_val = 2.4477;
    theta_grid = linspace(0,2*pi);
    phi = angle;
    h=corner(1);      %Centre of ellipse should be last x,y coords
    k=corner(2);  
    a=chisquare_val*sqrt(largest_val);
    b=chisquare_val*sqrt(smallest_val);
%     a=2*V(1)*5.991^0.5;
%     b=2*V(2)*5.991^0.5;
    
    % the ellipse in x and y coordinates 
    ellipse_x_r  = a*cos( theta_grid );
    ellipse_y_r  = b*sin( theta_grid );
    
    %Define a rotation matrix
    R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
    
    %let's rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r;ellipse_y_r]'*R;

    

    % Draw the error ellipse
%     plot(r_ellipse(:,1) + h,r_ellipse(:,2) + k,'-')
%     hold on;
end