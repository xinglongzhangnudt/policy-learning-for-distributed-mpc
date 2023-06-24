function [NIError,State,R_State]=draw_fig1_new_cent(ANN,ANN_u,ConHor_len,state_bound,NIError,State,R_State,Obstacle,k,Iterations_num,Umax1,Umin1)
global tau iter run;
persistent E U X J j PresentState PE  Present_x 
if(k==1&iter==1)
J=zeros(run,Iterations_num);
end
if(k==1)
    E=zeros(5,Iterations_num);
    j=0;
    U=zeros(2,Iterations_num);
    X=zeros(5,Iterations_num);
    PresentState=[0;0.9;1;0;0];
    PE=NIError;
    Present_x=[0;1;1;0;0.2];
end
PresentError=NIError;
timesteps=0;  
PresentState=State;
Present_x=R_State;
Umax=[5;5];Umin=[-5;-5];
while(timesteps<ConHor_len)
    %load  centralized_cost1.mat
    c_input=PresentError./state_bound;        
    ANN=NNProcess1(ANN,c_input);
    [dB_u]=Barriera_new_cen(Umax1,Umin1,ANN.NetworkOut);
     ANN_u=ANN_xProcess(ANN_u,dB_u);
    u=ANN.NetworkOut+ANN_u.NetworkOut;%+0*ANN_x1.NetworkOut;
     FutureState=robot(PresentState,u);
     Future_x=desired_position(Present_x);
     Thita=FutureState(4);
    T=[cos(Thita),sin(Thita),0,0,0;
              -sin(Thita),cos(Thita),0,0,0;
              0,0,1,0,0;
              0,0,0,1,0;
              0,0,0,0,1];
    FE=T*(Future_x-FutureState); 
  
    FutureError=FE;
    E(:,k+timesteps)=PE;
    X(:,k+timesteps)=PresentState;
    U(:,k+timesteps)=u;
    PresentError=FutureError;
    PresentState=FutureState;
    Present_x=Future_x;
    PE=FE;
    Q1=1*eye(5);
    R=0.1*eye(2);
    Q1(4,4)=2;
    Q1(5,5)=2;
    j=j+PresentError'*Q1*PresentError+u'*R*u;
    J(iter,k+timesteps)=j;
    timesteps=timesteps+1;
end
NIError=PE;
State=PresentState;
R_State=Present_x;

if(k+timesteps-1>=Iterations_num)
%      figure;
%      plot(1:size(E,2),E);
%     subplot(4,1,1);
%     plot(E1(1,:));
%     subplot(4,1,2);
%     plot(E1(2,:));
%     subplot(4,1,3);
%     plot(E1(3,:));
%     subplot(4,1,4);
%     plot(E1(4,:));
%     title('State error');
if iter==run
    pause();
end
%% plot path
%     figure;
%     plot(X(1,:),X(2,:),'b');
%     axis equal;
    %%
%     figure;
%     subplot(2,1,1);
%     plot(U1(1,:));
%     subplot(2,1,2);
%     plot(U1(2,:));
 %   title('Control input');
end