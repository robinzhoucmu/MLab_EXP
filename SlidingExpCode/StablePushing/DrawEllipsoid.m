function [h] = DrawEllipsoid(r)
maxd = 20;
step = maxd / 80;

[ xx, yy, zz ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);
shape = r(1) * xx.^2 + r(2) * yy.^2 + r(3) * zz.^2 + ...
        r(4) *  xx.* yy + r(5) * xx.* zz + r(6) * yy .* xx;
h = figure;

p = patch(isosurface(xx,yy,zz,shape,1));
set( p, 'FaceColor', 'g','FaceAlpha', 0.25, 'EdgeColor', 'none' );
view(-10, 20);
%view(3);
camlight
end

