function Data=circle_func(d,theta_sam,Obstacle)
Data=[];
for theta=theta_sam:theta_sam:2*pi
    x=d*cos(theta)+Obstacle(1);
    y=d*sin(theta)+Obstacle(2);
    Data=[Data,[x;y]];
end