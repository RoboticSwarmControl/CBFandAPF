function hCircles = drawcircles(hAxes,C,R)
    angle = 0:1:360;
    hCircles = gobjects(numel(R),1);
    for i = 1:numel(R)
        x = C(i,1) + R(i) * cosd(angle);
        y = C(i,2) + R(i) * sind(angle);
        hCircles(i) = fill(hAxes,x,y,"-k");
    end
end