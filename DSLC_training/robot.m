function FutureState=robot(x,u)
global tau;
v=x(4)+tau*u(1);
FutureState(1,1)=x(1)+tau*v*cos(x(3));
FutureState(2,1)=x(2)+tau*v*sin(x(3));
FutureState(3,1)=x(3)+tau*u(2);
FutureState(4,1)=v;
if(FutureState(3,1)>pi)
    FutureState(3,1)=FutureState(3,1)-2*pi;
elseif(FutureState(3,1)<-pi)
    FutureState(3,1)=FutureState(3,1)+2*pi;
end