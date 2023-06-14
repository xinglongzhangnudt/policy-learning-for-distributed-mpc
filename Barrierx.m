function [B_x]=Barrierx(PresentState,Obstacle)
global Data
  persistent pre1; 
  persistent pre2;
  distance = sqrt((PresentState(1)-Obstacle(1))^2+(PresentState(2)-Obstacle(2))^2);
  MaxControl_1=1;
  MaxControl_2=1;
  g_1max=1/MaxControl_1;
  g_2max=1/MaxControl_2;
  
  offset1=0.1;offset2=0.1;
%  
%% the recentered barrie function is -log(x^2+y^2-r^2)-log(R^2-r^2)+1/(x_c^2+y_c^2-r^2)*[2xc;2yc]'[x;y];
%  d1=2.5; radius=1.5;
%  offset1=0.5;offset2=0.1;
% posX=PresentState(1);
% posY=PresentState(2);
% [minindex,xc,yc]=find_minindex(Data,posX,posY);
% if distance>d1
%     B_x=[0;0];
% elseif (distance<d1 & distance^2>radius^2+(offset1)^2) 
%  %  pause(2);
%     B_x=(-1/(PresentState(1)^2+PresentState(2)^2-radius^2)*[2*PresentState(1);2*PresentState(2)]+1/(xc^2+yc^2-radius^2)*[2*xc;2*yc]);
% elseif (distance^2<=(radius^2+(offset1)^2))
%     B_x=-2/(radius^2+offset1^2)*[PresentState(1);PresentState(2)];
% end
%%
 B_x1=(g_1max/(1-g_1max*distance))*abs(PresentState(1)-Obstacle(1))/distance;
 B_x2=(g_2max/(1-g_2max*distance))*abs(PresentState(2)-Obstacle(2))/distance;
    if(distance>MaxControl_1+offset1 & distance<MaxControl_1+offset1+0.1)
      pre1=PresentState(1);
      pre2=PresentState(2);
  end
  if (distance>MaxControl_1+offset1);
     B_x1=(g_1max/(1-g_1max*distance))*abs(PresentState(1)-Obstacle(1))/distance;
    elseif (distance<=MaxControl_1+offset1);
     up=MaxControl_1+offset1;
     %k=(g_1max/(1-g_1max*up))^2;
     k=g_1max*(-1)*(up-g_1max*up^2)^(-2)*(1-2*g_1max*up)*abs(pre1-Obstacle(1));
     %k=g_1max*(-1)*(up-g_1max*up^2)^(-2)*(1-2*g_1max*up);
     b=(g_1max/(1-g_1max*up))*abs(PresentState(1)-Obstacle(1))/up;
     B_x1=k*(distance-up)+b;
    end
    
  if (distance>MaxControl_2+offset2);
     B_x2=(g_2max/(1-g_2max*distance))*abs(PresentState(2)-Obstacle(2))/distance;
    elseif (distance<=MaxControl_2+offset2);
     up=MaxControl_2+offset2;
     k=g_2max*(-1)*(up-g_2max*up^2)^(-2)*(1-2*g_2max*up)*abs(pre2-Obstacle(2));
     %k=g_2max*(-1)*(up-g_2max*up^2)^(-2)*(1-2*g_2max*up);
     b=(g_2max/(1-g_2max*up))*abs(PresentState(2)-Obstacle(2))/up;
     B_x2=k*(distance-up)+b;
    end
    B_x=[B_x1;B_x2];
     
     
%    distance = sqrt((PresentState(1)-Obstacle(1))^2+(PresentState(2)-Obstacle(2))^2);
%    MaxControl_1=1;
%    MaxControl_2=1; 
%    g_1max=1/MaxControl_1;
%    g_2max=1/MaxControl_2;
%    offset1=0.1;
% 
%   if (distance>MaxControl_1+offset1);
%      B_x1=(g_1max/(1-g_1max*distance));
%     elseif (distance<=MaxControl_1+offset1);
%      up=MaxControl_1+offset1;
%      %k=(up*(1-up)-abs(PresentState(1)-Obstacle(1))*(abs(PresentState(1)-Obstacle(1))/up-2*abs(PresentState(1)-Obstacle(1))))/(up*(1-up))^2;
%      %b=(g_1max/(1-g_1max*up))*abs(PresentState(1)-Obstacle(1))/up;
%      k=(g_1max/(1-g_1max*up))^2;
%      b=(g_1max/(1-g_1max*up));
%      B_x1=k*(distance-up)+b;
%     end
% 
%    
%    B_x=[B_x1;B_x1];
    