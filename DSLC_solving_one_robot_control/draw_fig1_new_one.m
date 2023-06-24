function [NIError,State,R_State]=draw_fig1_new_one(ANN1,ANN2,ANN_u1,ANN_u2,ConHor_len,state_bound1,state_bound2,NIError,State,R_State,Obstacle,k,Iterations_num,Umax1,Umin1,Umax2,Umin2)
global tau iter run;
persistent E U X  PresentState PE  Present_x j J
if(k==1&iter==1)
J=zeros(run,Iterations_num);
end
if(k==1)
    E=zeros(5,Iterations_num);
    j=0;
    U=zeros(2,Iterations_num);
    X=zeros(5,Iterations_num);
    Y=zeros(3,Iterations_num);
    PresentState=[0;0.9;1;0;0];
    PE=NIError;
    Present_x=[0;1.0;1;0;0.2];
end
PresentError=NIError;
timesteps=0;  
PresentState=State;
Present_x=R_State;
Umax=[5;5];Umin=[-5;-5];
while(timesteps<ConHor_len)
     c_input1=(PresentError(1:3))./state_bound1;        
     c_input2=PresentError(4:5)./state_bound2;        
     ANN1=NNProcess1(ANN1,[c_input1;c_input2]);% 
     ANN2=NNProcess1(ANN2,c_input2); %W_a*sigma_a     
     [dB_u1]=Barriera_new(Umax1,Umin1,ANN1.NetworkOut);
     [dB_u2]=Barriera_new(Umax2,Umin2,ANN2.NetworkOut);
     ANN_u1=ANN_xProcess(ANN_u1,dB_u1);
     ANN_u2=ANN_xProcess(ANN_u2,dB_u2);
    u1=ANN1.NetworkOut+ANN_u1.NetworkOut;%+0*ANN_x1.NetworkOut;
    u2=ANN2.NetworkOut+ANN_u2.NetworkOut;%+0*ANN_x2.NetworkOut;
     FutureState=robot(PresentState,[u1;u2]);
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
    U(:,k+timesteps)=[u1;u2];
    PresentError=FutureError;
    PresentState=FutureState;
    Present_x=Future_x;
    PE=FE;
    Q1=1*eye(5);
    Q1(4,4)=2;
    Q1(5,5)=2;
    R=0.1*eye(2);
    j=j+PresentError'*Q1*PresentError+[u1;u2]'*R*[u1;u2];
    J(iter,k+timesteps)=j;
    timesteps=timesteps+1;
end
NIError=PE;
State=PresentState;
R_State=Present_x;

if(k+timesteps-1>=Iterations_num)
%    figure;
%     subplot(4,1,1);
%     plot(1:size(E,2),E);
%     subplot(4,1,2);
%     plot(E1(2,:));
%     subplot(4,1,3);
%     plot(E1(3,:));
%     subplot(4,1,4);
%     plot(E1(4,:));
%    title('State error');
%% plot path
if iter==run
    pause();
end
%     figure;
%     plot(X(1,:),X(2,:),'b');
%     axis equal;
    %%
%     figure;
%     subplot(2,1,1);
%     plot(U1(1,:));
%     subplot(2,1,2);
%     plot(U1(2,:));
%    title('Control input');
end