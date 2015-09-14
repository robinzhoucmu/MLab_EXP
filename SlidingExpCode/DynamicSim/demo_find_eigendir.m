poly4_coeffs = [ 0.0012
    0.0015
    0.0038
   -0.0000
    0.0009
    0.0008
   -0.0001
    0.0010
    0.0024
    0.0028
    0.0096
    0.0060
    0.0022
   -0.0036
    0.0061];
pho = 0.05;

N1 = 200;
N2 = 200;
eps_parallel = 1e-3;
v = zeros(3,1);
ct = 0;
for theta = 0: 2*pi/N1: 2*pi
      for phi = 0: pi/N2: pi
        v(1) = cos(theta) * sin(phi);
        v(2) = sin(theta) * sin(phi);
        v(3) = cos(phi);
        F = FindForceGivenVel(v, poly4_coeffs, 'poly4');
        F_dir = F / norm(F);
        if (norm(F_dir - v) < eps_parallel)
            ct = ct + 1;
            v
            F
        end
      end
end
fprintf('Total eigen dir: %d\n', ct);