function [B_u]=Barriera_new(Umaxh,Umaxl,U)
    MaxControl_1=Umaxh(1);
    MinControl_1=Umaxl(1);
    MaxControl_2=Umaxh(2);
    MinControl_2=Umaxl(2);
    AproU1=U(1); AproU2=U(2); 
    offset1=0.1;offset2=0.1;
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
     if (AproU2>MinControl_2+offset2 && AproU2<MaxControl_2-offset2)
     B_u2=1/(MaxControl_2-AproU2)-1/(MinControl_2-AproU2)-1/MaxControl_2+1/MinControl_2;
    elseif (AproU2>=MaxControl_2-offset2)
     up=MaxControl_2-offset2;
     k=1/(MaxControl_2-up)^2-1/(MinControl_2-up)^2;
     b=1/(MaxControl_2-up)-1/(MinControl_2-up)-1/MaxControl_2+1/MinControl_2;
     B_u2=k*(AproU2-up)+b; 
    else(AproU2<=MinControl_2+offset2);
     down=MinControl_2+offset2;
     k=1/(MaxControl_2-down)^2-1/(MinControl_2-down)^2;
     b=1/(MaxControl_2-down)-1/(MinControl_2-down)-1/MaxControl_2+1/MinControl_2;
     B_u2=k*(AproU2-down)+b;
     end
     B_u=[B_u1;B_u2];
    