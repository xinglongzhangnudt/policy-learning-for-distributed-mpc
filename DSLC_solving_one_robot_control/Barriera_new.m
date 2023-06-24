function [B_u]=Barriera_new(Umaxh,Umaxl,U)
    MaxControl_1=Umaxh(1);
    MinControl_1=Umaxl(1);
    AproU1=U(1); 
    offset1=0.1;
    if (AproU1>MinControl_1+offset1 && AproU1<MaxControl_1-offset1)
     B_u1=1/(MaxControl_1-AproU1)-1/(MinControl_1-AproU1)-1/MaxControl_1+1/MinControl_1;
    elseif (AproU1>=MaxControl_1-offset1)
     up=MaxControl_1-offset1;
     k=1/(MaxControl_1-up)^2-1/(MinControl_1-up)^2;
     b=1/(MaxControl_1-up)-1/(MinControl_1-up)-1/MaxControl_1+1/MinControl_1;
     B_u1=k*(AproU1-up)+b; 
    else AproU1<=MinControl_1+offset1;
     down=MinControl_1+offset1;
     k=1/(MaxControl_1-down)^2-1/(MinControl_1-down)^2;
     b=1/(MaxControl_1-down)-1/(MinControl_1-down)-1/MaxControl_1+1/MinControl_1;
     B_u1=k*(AproU1-down)+b;
    end
     B_u=B_u1;
    