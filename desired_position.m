function Future_x=desired_position(x)
global tau;
vr=1;
wr=0;
FutureState(1,1)=x(1)+tau*vr*cos(x(3));
FutureState(2,1)=x(2)+tau*vr*sin(x(3));
FutureState(3,1)=x(3)+tau*wr;
FutureState(4,1)=vr;
if(FutureState(3,1)>pi)
    FutureState(3,1)=FutureState(3,1)-2*pi;
elseif(FutureState(3,1)<-pi)
    FutureState(3,1)=FutureState(3,1)+2*pi;
end
Future_x=[FutureState(1,1);FutureState(2,1);FutureState(3,1);FutureState(4,1)];