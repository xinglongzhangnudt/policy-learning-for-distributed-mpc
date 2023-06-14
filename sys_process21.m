function [MSJ2,MCJ2,MSJ21]=sys_process21(ep,u,PresentState1,PresentState2)
global tau;
ep1=ep(1);
ep2=ep(2);
ep3=ep(3);
ep4=ep(4);
u1=u(1);
u2=u(2);
wr=0;
vr=1;
v1=PresentState1(4);
thita2=PresentState2(3);
thita1=PresentState1(3);
MSJ2 = [       1, tau*u2, -tau*sin(ep3)*vr-tau*sin(thita1-thita2)*(v1),2*tau;
  -tau*u2,      1,  tau*cos(ep3)*vr+tau*cos(thita1-thita2)*(v1),0;
        0,      0,    1,0;
        0,0,0,1];
MCJ2 =[ 0,  ep2*tau;
    0, -ep1*tau;
    0,     -tau;
    -tau,0];
MSJ21 = [0, 0, tau*sin(thita1-thita2)*(v1),-tau*cos(thita1-thita2);
        0, 0, -tau*cos(thita1-thita2)*(v1),-tau*sin(thita1-thita2);
        0,0, 0,0;
        0,0,0,0];