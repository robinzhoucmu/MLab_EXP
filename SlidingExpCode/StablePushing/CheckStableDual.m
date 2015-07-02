function [flag_stable] = CheckStableDual(W, V)
NSegs = 50;
delta_theta = 2*pi/NSegs;
delta_phi = pi/NSegs;
flag_stable = 1;
theta = 0:delta_theta:2*pi;
phi = 0:delta_phi:pi;
% Outer product.
t1 = bsxfun(@times, cos(theta), sin(phi)');
t2 = bsxfun(@times, sin(theta), sin(phi)');
t3 = repmat(cos(phi)', [1 length(theta)]);
Y1 = [t1(:)';t2(:)';t3(:)'];
% Y1 = [bsxfun(@times, cos(theta), sin(phi)); ...
%      bsxfun(@times, sin(theta), sin(phi)); ...
%      cos(phi)];
ind = Y1' * V < 0;
Y = Y1(:,ind);
i = 1;
while flag_stable && i <= size(Y,2)
   [flag_exist, A] = FindSeparatingPlane(W, V, Y(:,i));
    if (flag_exist)
        flag_stable = 0;
        A
        V
        break;
    end
    i = i+1;
end
% for theta = 0:delta_theta:2*pi
%     if (flag_stable)
%         for phi = 0:delta_phi:pi
%             %theta, phi
%             if (flag_stable)
%                 y = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
%                 if (y'*V < 0)
%                     [flag_exist, A] = FindSeparatingPlane(W, V, y);
%                     if (flag_exist)
%                         flag_stable = 0;
%                         A
%                         V
%                         break;
%                     end
%                 end
%             end
%         end
%     end    
% end

end

