clear all;close all;
global tau iter run;
tau=0.05;  
Gamma=0.95;   %%discount factor
state_bound1=[2;2;2];  %X,Y,V
state_bound2=[2;2]; %THETA
Iterations_num=1600;
MaxTrials=30; 
ConHor_len=1; %% Control Horizon length
PreHor_len=20; %% Predictive Horizon length
center_barrier=2.5;
Obstacle=[3000;1];
%% Initialization
ANN1=CreateANN1(5,1);
ANN2=CreateANN1(2,1);
ANN_u1=CreateANN_x(1,1);
ANN_u2=CreateANN_x(1,1);
ANN_x2=CreateANN_x(2,1);
CNN1=CreateCNN1(5,5);
CNN2=CreateCNN1(2,2);
%%
State=[0;0.9;1;0;0];
R_State=[0;1.0;1;0;0.2];

Thita=State(3);
T=[cos(Thita),sin(Thita),0,0,0;
    -sin(Thita),cos(Thita),0,0,0;
    0,0,1,0,0;
    0,0,0,1,0;
    0, 0,0,0,1];
NIError=T*(R_State-State);
mu=0.02;
Q1=1*eye(5);
Q2=1*eye(2);
R=0.1*eye(2);
J=[];J_1=[];J_1d=[];
Umax1=[5];Umin1=[-5]; % enlarge the control constraint for collision avoidance
Umax2=[5];Umin2=[-5]; % enlarge the control constraint for collision avoidance
flag1=0;
tic
%% Main loop  
run=200;
NIError0=NIError;
State0=State;
R_State0=R_State;
Jc=zeros(run,Iterations_num);
%% 
for iter=1:run
disp(['run=',num2str(iter)]);
NIError=NIError0;
State=State0;
R_State=R_State0;
for k=1:ConHor_len:Iterations_num
    disp(['iteration=',num2str(k)]);
    RealError=NIError;
    RealState=State;
    Present_rx=R_State;
    f=1;
    %% Determine whether the learning is successful in the current prediction time domain
    while(f>=1)
        PresentError=RealError;
        PresentState=RealState;
        Present_x=Present_rx;
        Err=0;
        timesteps=0;  j=0;ANN1d=[];ANN_u1d=[];ANN_x1d=[];
        while(timesteps<PreHor_len)
            c_input1=PresentError(1:3)./state_bound1;        
            c_input2=PresentError(4:5)./state_bound2;        
            ANN1=NNProcess1(ANN1,[c_input1;c_input2]);
            ANN2=NNProcess1(ANN2,c_input2); %W_a*sigma_a
            distance = sqrt((PresentState(1)-Obstacle(1))^2+(PresentState(2)-Obstacle(2))^2);
            if distance<1%;%0.2+0.03
                flag1=1;
            end
            u1_v=ANN1.NetworkOut;
            u2_v=ANN2.NetworkOut;
            [dB_u1]=Barriera_new(Umax1,Umin1,u1_v);
            [dB_u2]=Barriera_new(Umax2,Umin2,u2_v);

            ANN_u1=ANN_xProcess(ANN_u1,dB_u1);%W_a*B_x'
            ANN_u2=ANN_xProcess(ANN_u2,dB_u2);
            u1=ANN1.NetworkOut+ANN_u1.NetworkOut;%+ANN_x1.NetworkOut;
            u2=ANN2.NetworkOut+ANN_u2.NetworkOut;%+ANN_x2.NetworkOut;
            FutureState=robot(PresentState,[u1;u2]);
            Future_x=desired_position(Present_x);
           [MSJ,MCJ]=sys_process12(PresentError,[u1;u2]);
           Thita=FutureState(4);
           T=[cos(Thita),sin(Thita),0,0,0;
              -sin(Thita),cos(Thita),0,0,0;
              0,0,1,0,0;
              0,0,0,1,0;
              0,0,0,0,1];
            FutureError=T*(Future_x-FutureState);
%  
            f_input1=FutureError(1:3)./state_bound1;
            f_input2=FutureError(4:5)./state_bound2;
            dR_dZ1=2*Q1*PresentError;%+mu*[dB_x1;0;0;0;0;0;0];         
            dR_dZ2=2*Q2*PresentError(4:5);%+mu*[dB_x2;0;0;0;0;0;0]; 
 %% CNN output
                CNN1=NNProcess1(CNN1,[f_input1;f_input2]);
                CNN2=NNProcess1(CNN2,f_input2);  
                FutureLambda1=CNN1.NetworkOut;
                FutureLambda2=CNN2.NetworkOut;
                CNN1=NNProcess1(CNN1,[c_input1;c_input2]); 
                CNN2=NNProcess1(CNN2,c_input2); 
                PresentLambda1=CNN1.NetworkOut;%+[CNN_x1.NetworkOut;0;0;0;0;0;0]; 
                PresentLambda2=CNN2.NetworkOut;%+[CNN_x2.NetworkOut;0;0;0;0;0;0];
  %% Update ANN and CNN
            ANNError1=2*(R(1))*u1+Gamma*(MCJ(:,1)'*FutureLambda1)+mu*dB_u1;  %              % calculate ANN error signal 
            ANNError2=2*(R(2))*u2+Gamma*(MCJ(4:5,2)'*FutureLambda2)+mu*dB_u2; %mu*dB_u2+
            ANN1=NNTrain1(ANN1,ANNError1);  % train ANN
            ANN2=NNTrain1(ANN2,ANNError2);
            ANN_u1=ANN_xTrain(ANN_u1,ANNError1); 
            ANN_u2=ANN_xTrain(ANN_u2,ANNError2);
            CNNTarget1=dR_dZ1+Gamma*MSJ'*FutureLambda1; % estimate of lambda(t)   
            CNNTarget2=dR_dZ2+Gamma*MSJ(4:5,4:5)'*FutureLambda2;
            CNNError1=PresentLambda1-CNNTarget1;
            CNNError2=PresentLambda2-CNNTarget2;

            CNN1=NNTrain1(CNN1,CNNError1);   % train CNN
            CNN2=NNTrain1(CNN2,CNNError2);
            PresentState=FutureState;
            Err=Err+0.5*sum(PresentError.^2);
            PresentError=FutureError;
            Present_x=Future_x;
            timesteps=timesteps+1; 
        end
        Err=Err/PreHor_len;
       %% Determine whether learning is successful whether learning is successful
        if(timesteps==PreHor_len)  %% success
            [NIError,State,R_State]=draw_fig1_new_one(ANN1,ANN2,ANN_u1,ANN_u2,ConHor_len,state_bound1,state_bound2,NIError,State,R_State,Obstacle,k,Iterations_num,Umax1,Umin1,Umax2,Umin2);
            f=0;
        else                       %% fail
            f=f+1;
        end
       %% If the learning fails for MaxTrials times in this time domain, reinitialize the network
        if(f>MaxTrials) 
            f=1;
        end
    end
end
if flag1==1
    disp(['Training complete']);
    break
end
end
toc
JL=sum(Jc')