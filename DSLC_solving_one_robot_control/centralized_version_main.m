clear all;close all;
global tau Data iter run;
tau=0.05;  
Gamma=0.95;   %%  discount factor
state_bound=[2;2;2;2;2];  %X,Y,V theta,w
Iterations_num=1600;
MaxTrials=30; 
ConHor_len=1; %% Control Horizon length
PreHor_len=20; %% Predictive Horizon length
center_barrier=2.5;
Obstacle=[3000;1];% no collision
Data=circle_func(center_barrier,0.1,Obstacle);
%% Initialization
% load safe_actor_critic.mat;
ANN=CreateANN1(5,2);
ANN_u=CreateANN_x(2,2);
CNN=CreateCNN1(5,5);
%%
State=[0;0.9;1;0;0] ;
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
R=0.1*eye(2);
Q1(4,4)=2;
Q1(5,5)=2;
J=[];J_1=[];J_1d=[];
Umax1=[5;5];Umin1=[-5;5]; % enlarge the control constraint to for collision avoidance
Umax2=[5];Umin2=[-5]; % enlarge the control constraint to for collision avoidance
OO=zeros(4,4);
flag1=0;
tic
%% Main loop  
run=200;
NIError0=NIError;
State0=State;
R_State0=R_State;
J=zeros(run,Iterations_num);
%% runing the iteration
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
    f=1;Cost1=0;Cost1d=0;
    %% Determine whether the learning is successful in the current prediction time domain
    while(f>=1)
        PresentError=RealError;
        PresentState=RealState;
        Present_x=Present_rx;
        Err=0;
        timesteps=0;  j=0;ANN1d=[];ANN_u1d=[];ANN_x1d=[];
        while(timesteps<PreHor_len)
            c_input=PresentError./state_bound;        
            ANN=NNProcess1(ANN,c_input);% 
            distance = sqrt((PresentState(1)-Obstacle(1))^2+(PresentState(2)-Obstacle(2))^2);
            if distance<1
                flag1=1;
            end
             u_v=ANN.NetworkOut;
             [dB_u]=Barriera_new_cen(Umax1,Umin1,u_v);
             ANN_u=ANN_xProcess(ANN_u,dB_u);%W_a*B_x'
             u=ANN.NetworkOut+ANN_u.NetworkOut;%+ANN_x1.NetworkOut;
    
            FutureState=robot(PresentState,u);
            Future_x=desired_position(Present_x);
           [MSJ,MCJ]=sys_process12(PresentError,u);
           Thita=FutureState(4);
           T=[cos(Thita),sin(Thita),0,0,0;
              -sin(Thita),cos(Thita),0,0,0;
              0,0,1,0,0;
              0,0,0,1,0;
              0,0,0,0,1];
            FutureError=T*(Future_x-FutureState);   
            f_input=FutureError./state_bound;
            dR_dZ=2*Q1*PresentError;%+mu*[dB_x1;0;0;0;0;0;0];         
 %% CNN output
                CNN=NNProcess1(CNN,f_input);
                FutureLambda=CNN.NetworkOut;
                CNN=NNProcess1(CNN,c_input); 
                PresentLambda=CNN.NetworkOut;
  %% Update ANN and CNN
            ANNError=2*(R)*u+Gamma*(MCJ'*FutureLambda)+mu*dB_u;              % calculate ANN error signal 
            ANN=NNTrain1(ANN,ANNError);  % train ANN
            CNNTarget=dR_dZ+Gamma*MSJ'*FutureLambda; % estimate of lambda(t)   
            CNNError=PresentLambda-CNNTarget;
            CNN=NNTrain1(CNN,CNNError);   % train CNN
            PresentState=FutureState;
            Err=Err+0.5*sum(PresentError.^2);
            PresentError=FutureError;
            Present_x=Future_x;
            timesteps=timesteps+1; 
        end
        Err=Err/PreHor_len;
       %% Determine whether learning is successful whether learning is successful
        if(timesteps==PreHor_len)  %% success
            [NIError,State,R_State]=draw_fig1_new_cent(ANN,ANN_u,ConHor_len,state_bound,NIError,State,R_State,Obstacle,k,Iterations_num,Umax1,Umin1);
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