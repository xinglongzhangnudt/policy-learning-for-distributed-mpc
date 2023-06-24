function [MSJ,MCJ]=sys_process12(ep,u)
global tau;
ep1=ep(1);
ep2=ep(2);
ep3=ep(3);
ep4=ep(4);
ep5=ep(5);
u1=u(1);
u2=u(2);
wr=0.08;
vr=0.72;

% 
% FutureState1=[ ep1 + tau*(ep2*(wr-ep5) -vr+ep3+ cos(ep4)*(vr));
%                ep2 + tau*(sin(ep4)*(vr)-ep1*(wr-ep5));
%                ep3 - tau*(u1);
%                ep4 + tau*ep5;
%                ep5 + tau*(-u2)];
MSJ = [1,  tau*(wr-ep5),tau,-tau*sin(ep4)*vr,-tau*ep2;
       -(wr-ep5),   1,   0,  tau*cos(ep4)*vr,tau*ep1;
        0, 0, 1,0,0;
        0,0,0,1,tau;
        0 0 0 0 1];
MCJ =[0,  0;
     0,    0;
     -tau, 0;
     0,0;
     0, -tau];