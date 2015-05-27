close all;
r = rand(15,1)
%r(1) = 10; 
maxd = 10;
axis equal;
step = maxd / 50;
[ xx, yy, zz ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);

shape = r(1) * xx.^4 + r(2) * yy.^4 + r(3)* zz.^4 + ...
r(4) *(xx.^2).* (yy.^2) + r(5) *(xx.^2) .* (zz.^2) + r(6) * (yy.^2) .* (zz.^2) + ...
r(7) * xx .* (yy.^3) + r(8) * xx .* (zz.^3)  + ...
r(9) * yy .* (xx.^3) + r(10) * yy .* (zz.^3) + ...
r(11) * zz .* (xx.^3) + r(12) * zz .* (yy.^3) + ...
r(13) * xx .* yy .* (zz.^2) + r(14) * xx .* (yy.^2) .* zz +  r(15) * (xx.^2) .* yy.^2 .* zz;

%shape = r(1) * xx.^4 + r(2) * yy.^4 + r(3)* zz.^4 + r(4) *(xx.^2).* (yy.^2) + r(5) *(xx.^2) .* (zz.^2) + r(6) * (yy.^2) .* (zz.^2);

%shape = r(1) * xx.^2 + r(2) * yy.^2 + r(3)* zz.^2 + r(4) *(xx.^1).* (yy.^1) + r(5) *(xx.^1) .* (zz.^1) + r(6) * (yy.^1) .* (zz.^1);


subplot(2,1,1);
p = patch(isosurface(xx,yy,zz,shape,1));
set( p, 'FaceColor', 'g', 'EdgeColor', 'y' );
view(3);
camlight

subplot(2,1,2);
p = patch(isosurface(xx,yy,zz,shape,10));
set( p, 'FaceColor', 'g', 'EdgeColor', 'y' );
view(3);
camlight