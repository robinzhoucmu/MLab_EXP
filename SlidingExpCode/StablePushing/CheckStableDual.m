function [flag_stable] = CheckStableDual(W, V)
NSegs = 20;
delta_theta = 2*pi/NSegs;
delta_phi = pi/NSegs;
flag_stable = 1;
for theta = 0:delta_theta:2*pi
    if (flag_stable)
        for phi = 0:delta_phi:pi
            %theta, phi
            if (flag_stable)
                y = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
                if (y'*V < 0)
                    [flag_exist, A] = FindSeparatingPlane(W, V, y);
                    if (flag_exist)
                        flag_stable = 0;
                        A
                        V
                        break;
                    end
                end
            end
        end
    end    
end

end

