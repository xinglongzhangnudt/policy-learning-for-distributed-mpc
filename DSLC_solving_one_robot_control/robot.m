function FutureState=robot(x,u)
global tau;
% v=x(3)+tau*u(1);
% w=x(5)+tau*u(2);
FutureState(1,1)=x(1)+tau*x(3)*cos(x(4));
FutureState(2,1)=x(2)+tau*x(3)*sin(x(4));
FutureState(3,1)=x(3)+tau*u(1);
FutureState(4,1)=x(4)+tau*x(5);
FutureState(5,1)=x(5)+tau*u(2);
if(FutureState(4,1)>pi)
    FutureState(4,1)=FutureState(4,1)-2*pi;
elseif(FutureState(4,1)<-pi)
    FutureState(4,1)=FutureState(4,1)+2*pi;
end