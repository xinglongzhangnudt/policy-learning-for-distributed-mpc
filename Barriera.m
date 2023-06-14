function [B_u]=Barriera(Umaxh,Umaxl,U)
  MaxControl_1=Umaxh(1);
  MinControl_1=Umaxl(1);
  MaxControl_2=Umaxh(2);
  MinControl_2=Umaxl(2);
  AproU1=U(1); AproU2=U(2);
  
  g_1max=1/MaxControl_1;
  g_1min=1/MinControl_1;
  g_2max=1/MaxControl_2;
  g_2min=1/MinControl_2;
  
  offset1=0.1;offset2=0.1;
    if (AproU1>MinControl_1+offset1 & AproU1<MaxControl_1-offset1);
     B_u1=g_1max/(1-g_1max*AproU1)+g_1min/(1-g_1min*AproU1)-g_1max-g_1min;
    elseif (AproU1>=MaxControl_1-offset1);
     up=MaxControl_1-offset1;
     k=(g_1max/(1-g_1max*up))^2+(g_1min/(1-g_1min*up))^2;
     b=g_1max/(1-g_1max*up)+g_1min/(1-g_1min*up)-g_1max-g_1min;
     B_u1=k*(AproU1-up)+b; 
    else(AproU1<=MinControl_1+offset1);
     down=MinControl_1+offset1;
     k=((g_1max/(1-g_1max*down))^2+(g_1min/(1-g_1min*down))^2);
     b=(g_1max/(1-g_1max*down)+g_1min/(1-g_1min*down)-g_1max-g_1min);
     B_u1=k*(AproU1-down)+b;
    end
     if (AproU2>MinControl_2+offset2 & AproU2<MaxControl_2-offset2);
     B_u2=g_2max/(1-g_2max*AproU2)+g_2min/(1-g_2min*AproU2)-g_2max-g_2min;
    elseif (AproU2>=MaxControl_2-offset2);
     up=MaxControl_2-offset2;
     k=(g_2max/(1-g_2max*up))^2+(g_2min/(1-g_2min*up))^2;
     b=g_2max/(1-g_2max*up)+g_2min/(1-g_2min*up)-g_2max-g_2min;
     B_u2=k*(AproU2-up)+b; 
    else(AproU2<=MinControl_2+offset2);
     down=MinControl_2+offset2;
     k=((g_2max/(1-g_2max*down))^2+(g_2min/(1-g_2min*down))^2);
     b=(g_2max/(1-g_2max*down)+g_2min/(1-g_2min*down)-g_2max-g_2min);
     B_u2=k*(AproU2-down)+b;
     end
     B_u=[B_u1;B_u2];
    