function [B_u]=A_new(Umaxh,Umaxl,U)
    MaxControl_1=Umaxh(1);
    MinControl_1=Umaxl(1);
    MaxControl_2=Umaxh(2);
    MinControl_2=Umaxl(2);
    AproU1=U(1); AproU2=U(2); 
    offset1=0.1;offset2=0.1;
 AproU1=0;AproU2=0;

    if (AproU1>MinControl_1+offset1 & AproU1<MaxControl_1-offset1)
        B_u1=-log(MaxControl_1-AproU1)-log(AproU1-MinControl_1);%-log(MaxControl_1)+log(MinControl_1)
        B_u1=B_u1-(-log(MaxControl_1-0)-log(0-MinControl_1));
     elseif (AproU1>=MaxControl_1-offset1)
     up=MaxControl_1-offset1;
     k1=(MaxControl_1-up)^(-2)-(up-MinControl_1)^(-2);
     k2=1/(MaxControl_1-up)-1/(up-MinControl_1);
     b2=-log(MaxControl_1-up)-log(up-MinControl_1);
     B_u1=1/2*k1*(AproU1-up)^2+k2*(AproU1-up)+b2; 
    else AproU1<=MinControl_1+offset1;
     down=MinControl_1+offset1;
     k1=(MaxControl_1-down)^(-2)-(down-MinControl_1)^(-2);
     k2=1/(MaxControl_1-down)-1/(down-MinControl_1);
     b2=-log(MaxControl_1-down)-log(down-MinControl_1);
     B_u1=-1/2*k1*(down-AproU1)^2-k2*(down-AproU1)+b2; 
   end

     if (AproU2>MinControl_2+offset2 && AproU2<MaxControl_2-offset2)
     B_u2=-log(MaxControl_2-AproU2)-log(AproU2-MinControl_2);%-log(MaxControl_1)+log(MinControl_1)
     B_u2=B_u2-(-log(MaxControl_2-0)-log(0-MinControl_2));
    elseif (AproU2>=MaxControl_2-offset2)
     up=MaxControl_2-offset2;
     k1=(MaxControl_2-up)^(-2)-(up-MinControl_2)^(-2);
     k2=1/(MaxControl_2-up)-1/(up-MinControl_2);
     b2=-log(MaxControl_2-up)-log(up-MinControl_2);
     B_u2=1/2*k1*(AproU2-up)^2+k2*(AproU2-up)+b2; 
    else(AproU2<=MinControl_2+offset2);
     down=MinControl_2+offset2;
     k1=(MaxControl_2-down)^(-2)-(down-MinControl_2)^(-2);
     k2=1/(MaxControl_2-down)-1/(down-MinControl_2);
     b2=-log(MaxControl_2-down)-log(down-MinControl_2);
     B_u2=-1/2*k1*(down-AproU2)^2-k2*(down-AproU2)+b2; 
     end
     B_u=[B_u1+B_u2];
    