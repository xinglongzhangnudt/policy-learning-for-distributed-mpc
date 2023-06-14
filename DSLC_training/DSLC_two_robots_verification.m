clear all;close all;
global tau iter;
load P_VALUE_two.mat % Load the given .mat files
load K_two.mat
load cost_LQR1_verification.mat
J_3d=J_1d;
%% Initialization
% Initialize constants, including control horizon, prediction horizon, barrier centers, obstacles, etc.
[tau,Gamma,Iterations_num,MaxTrials,ConHor_len,PreHor_len,center_barrier,Obstacle,mu,Q,R,run,scale,Umax,Umin] = set_constants_no_obs();
agent=2; % Set agent count to 2
[ANN,ANN_u,ANN_x,CNN,CNN_x,state_bound] = createNetworks(agent); % Create network structures for each agent
[State,R_State,V,NIError] =Initial_state_calculation_no_obs; % Compute the initial state
J=[];J_1=[];J_1d=[]; % Preallocate memory for performance measurement
flag1=0; % Flag used to indicate if an obstacle is too close
run=1;
J=zeros(run,Iterations_num);
const=0.1;
scale=50;
tic % Start timing the execution
%% Main loop
for iter=1:run
   disp(['run=',num2str(iter)]);
Theorem4_1_robot{1}=[];Theorem4_2_robot{1}=[];
Theorem4_1_robot{2}=[];Theorem4_2_robot{2}=[];
for k=1:ConHor_len:Iterations_num % Loop over control horizon
    disp(['iteration=',num2str(k)]); % Display the current run number
       RealError=NIError;
       RealState=State;
       Present_rx=R_State;
    f=1;CosT{1}=0;CosT_d{1}=0;
    %% Determine whether the learning is successful in the current prediction time domain
    while(f>=1)
        PresentError=RealError;
        PresentState=RealState;
        Present_x=Present_rx;
        Err1=0;Err2=0;
        timesteps=0;  j=0;ANN_d{1}=[];ANN_u_d{1}=[];ANN_x_d{1}=[];
        while(timesteps<PreHor_len)
            for i=1:agent
            c_input{i}=PresentError{i}./state_bound{i};
            end    
         
            ANN{1}=NNProcess1(ANN{1},[c_input{1};c_input{2}]);%æ¿?æ´»å‡½æ•?   
            ANN{2}=NNProcess1(ANN{2},[c_input{2};c_input{1}]); %W_a*sigma_a
            
            for i=1:agent
            ANN_eta{i}=tanh(ANN{i}.NetworkIn);
            end
            
            distance = sqrt((PresentState{1}(1)-Obstacle(1))^2+(PresentState{1}(2)-Obstacle(2))^2);
            if distance<1%;%0.2+0.03
                flag1=1;
            end
%% ANN output   
            for i=1:agent
            u_v{i}=ANN{i}.NetworkOut;
            [dB_u{i}]=Barriera(Umax{i},Umin{i},u_v{i});
            [dB_x{i}]=Barrierx(PresentState{i},Obstacle);
            [B_x{i}]=B_new(PresentState{i},Obstacle);
            [B_u{i}]=A_new(Umax{i},Umin{i},u_v{i});

            ANN_u{i}=ANN_xProcess(ANN_u{i},dB_u{i});%W_a*B_x'
            ANN_x{i}=ANN_xProcess(ANN_x{i},dB_x{i});%W_a*B_u'
            u{i}=ANN{i}.NetworkOut+ANN_u{i}.NetworkOut+ANN_x{i}.NetworkOut;
            end  
 %%   stabilizing learned policy for verification        
            if timesteps==0          
            for i=1:agent
              ANN_d{i}=ANN{i};ANN_u_d{i}=ANN_u{i};ANN_x_d{i}=ANN_x{i};
              PresentState_d{i}=PresentState{i};
              PresentError_d{i}=PresentError{i};
            end
            const=const-0.0002;
            end
              if const<0
                const=0;
              end
            
            for i=1:agent
            [B_x_d{i}]=B_new(PresentState_d{i},Obstacle);
            c_input_d{i}=PresentError_d{i}./state_bound{i};  
            end
            
            ANN_d{1}=NNProcess1(ANN_d{1},[c_input_d{1};c_input_d{2}]);%æ¿?æ´»å‡½æ•?   
            ANN_d{2}=NNProcess1(ANN_d{2},[c_input_d{2};c_input_d{1}]);%æ¿?æ´»å‡½æ•?   
            
            for i=1:agent
            u_v_d{i}=ANN_d{i}.NetworkOut;
            [B_u_d{i}]=A_new(Umax{i},Umin{i},u_v_d{i});
            [dB_u_d{i}]=Barriera(Umax{i},Umin{i},u_v_d{i});
            [dB_x_d{i}]=Barrierx(PresentState_d{i},Obstacle);
            ANN_u_d{i}=ANN_xProcess(ANN_u_d{i},dB_u_d{i});%W_a*B_x'
            ANN_x_d{i}=ANN_xProcess(ANN_x_d{i},dB_x_d{i});%W_a*B_u'
            end
            
            bias=0.7^(k^0.8)+const;
            if k<=PreHor_len&&timesteps<PreHor_len-k+1
                u1dnom=(KN1-5)*[PresentError_d{1};PresentError_d{2}]+bias; 
                u2dnom=(KN2-5)*[PresentError_d{1};PresentError_d{2}]+bias;
             else
                u1dnom=(KN1-5)*[PresentError_d{1};PresentError_d{2}]+bias;
                u2dnom=(KN2-5)*[PresentError_d{1};PresentError_d{2}]+bias;
            end
  %% Update State
             Future_x=desired_position(Present_x);
            for i=1:agent
             udnom{i}=ANN_d{i}.NetworkOut+ANN_u_d{i}.NetworkOut+ANN_x_d{i}.NetworkOut+bias; %lerned
             u_d{i}=udnom{i}+bias;
           
            FutureState{i}=robot(PresentState{i},u{i});
            FutureState_d{i}=robot(PresentState_d{i},u_d{i});
            end
            
           [MSJ1,MCJ1,MSJ12]=sys_process12(PresentError{1},u{1},PresentState{1},PresentState{2});
           [MSJ2,MCJ2,MSJ21]=sys_process21(PresentError{2},u{2},PresentState{1},PresentState{2});
           
           for i=1:agent
           Thita{i}=FutureState{i}(3);
           Thita_d{i}=FutureState_d{i}(3);
           T{i} = calc_T(Thita{i}); 
           T_d{i} = calc_T(Thita_d{i}); 
           end
           
            FutureError{1}=T{1}*((Future_x-FutureState{1})+(V*(FutureState{2}-FutureState{1})+[0;1;0;0]));
            FutureError{2}=T{2}*((Future_x-FutureState{2}+[0;-1;0;0])+(V*(FutureState{1}-FutureState{2})+[0;-1;0;0]));
            FutureError_d{1}=T_d{1}*((Future_x-FutureState_d{1})+(V*(FutureState{2}-FutureState_d{1})+[0;1;0;0]));
            FutureError_d{2}=T_d{2}*((Future_x-FutureState_d{2}+[0;-1;0;0])+(V*(FutureState_d{1}-FutureState_d{2})+[0;-1;0;0]));

%% CNN output         
            for i=1:agent
            f_inpuT{i}=FutureError{i}./state_bound{i};
            end
            
            dR_dZ{1}=2*Q*[PresentError{1};PresentError{2}]+mu*[dB_x{1};0;0;0;0;0;0];         
            dR_dZ{2}=2*Q*[PresentError{2};PresentError{1}]+mu*[dB_x{2};0;0;0;0;0;0]; 
            cosT{1}=2*[PresentError{1};PresentError{2}]'*Q*[PresentError{1};PresentError{2}]+mu*B_x{1}+mu*B_u{1}+u{1}'*R*u{1};
            CosT{1}=CosT{1}+cosT{1};
            cosT_d{1}=2*[PresentError_d{1};PresentError_d{2}]'*Q*[PresentError_d{1};PresentError_d{2}]+mu*B_x_d{1}+mu*B_u_d{1}+u_d{1}'*R*u_d{1};
            CosT_d{1}=CosT_d{1}+cosT_d{1}; 

                CNN{1}=NNProcess1(CNN{1},[f_inpuT{1};f_inpuT{2}]);
                CNN{2}=NNProcess1(CNN{2},[f_inpuT{2};f_inpuT{1}]);  
              
              for i=1:agent
                CNN_eta_f{i}=tanh(CNN{i}.NetworkIn);
                [dB_x_f{i}]=Barrierx(FutureState{i},Obstacle);
                [B_x_f{i}]=B_new(FutureState{i},Obstacle);
                CNN_x{i}=CNN_xProcess(CNN_x{i},dB_x_f{i});%barrier_x
                FutureLambda{i}=CNN{i}.NetworkOut+[CNN_x{i}.NetworkOut;0;0;0;0;0;0];  %CNN_x{2}.NetworkOut
              end
              
                if timesteps==PreHor_len-1
                    FutureLambda{1}=[2*P1/scale*FutureError{1};zeros(4,1)];
                    FutureLambda{2}=[zeros(4,1);2*P2/scale*FutureError{2}];
                end
                CNN{1}=NNProcess1(CNN{1},[c_input{1};c_input{2}]);
                CNN{2}=NNProcess1(CNN{2},[c_input{2};c_input{1}]); 
                
                for i=1:agent
                CNN_x{i}=CNN_xProcess(CNN_x{i},dB_x{i});
                PresentLambda{i}=CNN{i}.NetworkOut+[CNN_x{i}.NetworkOut;0;0;0;0;0;0]; 
                CNN_eta{i}=tanh(CNN{i}.NetworkIn);
                end
                
  %% Update ANN and CNN
            ANNError{1}=2*(R)*u{1}+mu*dB_u{1}+Gamma*(MCJ1'*FutureLambda{1}(1:4)+MCJ1'*FutureLambda{2}(5:8));                % calculate ANN error signal 
            ANNError{2}=2*(R)*u{2}+mu*dB_u{2}+Gamma*(MCJ2'*FutureLambda{2}(1:4)+MCJ2'*FutureLambda{1}(5:8));
            
            for i=1:agent
            ANN{i}=NNTrain1(ANN{i},ANNError{i});  % train ANN
            ANN_u{i}=ANN_xTrain(ANN_u{i},ANNError{i}); 
            ANN_x{i}=ANN_xTrain(ANN_x{i},ANNError{i}); 
            end
            
            CNNTargeT{1}=dR_dZ{1}+Gamma*[MSJ1',MSJ21';MSJ12',MSJ2']*FutureLambda{1}; % estimate of lambda(t)   
            CNNTargeT{2}=dR_dZ{2}+Gamma*[MSJ2',MSJ12;MSJ21',MSJ1']*FutureLambda{2};
            
            for i=1:agent
            CNNError{i}=PresentLambda{i}-CNNTargeT{i};
            end
            
            F1=[MSJ1',MSJ21';MSJ12',MSJ2'];
            
            [Tau_42a_roboT{1}]=T_42a(CNN_eta{1},CNN_eta_f{1},F1,CNN{1}.LR,B_x{1},B_x_f{1},CNN_x{1}.LR);
            Theorem4_1_robot{1}=[Theorem4_1_robot{1},Tau_42a_roboT{1}];
            
            [Tau_42b_roboT{1}]=T_42b(R,ANN{1}.LR,ANN_x{1}.LR,ANN_u{1}.LR,ANN_eta{1},dB_x{1},dB_u{1});
            Theorem4_2_robot{1}=[Theorem4_2_robot{1},Tau_42b_roboT{1}];
            
            [Tau_42a_roboT{2}]=T_42a(CNN_eta{2},CNN_eta_f{2},F1,CNN{2}.LR,B_x{2},B_x_f{2},CNN_x{2}.LR);
            Theorem4_1_robot{2}=[Theorem4_1_robot{2},Tau_42a_roboT{2}];
            
            [Tau_42b_roboT{2}]=T_42b(R,ANN{2}.LR,ANN_x{2}.LR,ANN_u{2}.LR,ANN_eta{2},dB_x{2},dB_u{2});
            Theorem4_2_robot{2}=[Theorem4_2_robot{2},Tau_42b_roboT{2}];
            
            for i=1:agent
            CNN{i}=NNTrain1(CNN{i},CNNError{i});   % train CNN
            CNN_x{i}=CNN_xTrain(CNN_x{i},CNNError{i}(1:2));
            end
            
            PresentState=FutureState;
            PresentState_d=FutureState_d;
            Err1=Err1+0.5*sum(PresentError{1}.^2);
            Err2=Err2+0.5*sum(PresentError{2}.^2);
            PresentError=FutureError;
            PresentError_d=FutureError_d;
            Present_x=Future_x;
            timesteps=timesteps+1; 
            if timesteps==2
            j=j+[PresentError{1};PresentError{2}]'*Q*[PresentError{1};PresentError{2}]+u{1}'*R*u{1}+[PresentError{2};PresentError{1}]'*Q*[PresentError{2};PresentError{1}]+u{2}'*R*u{2};
            end
            end
        J(iter,k)=j;
        J_1=[J_1,CosT{1}]; J_1d=[J_1d,CosT_d{1}];
        Err1=Err1/PreHor_len;
        Err2=Err2/PreHor_len;
       %% Determine whether learning is successful whether learning is successful
        if(timesteps==PreHor_len)  %% success
            [NIError,State,R_State]=draw_fig1_new_no_obs(ANN,ANN_x,ANN_u,ConHor_len,state_bound,NIError,State,R_State,Obstacle,k,Iterations_num);
            
            f=0;
        else                       %% fail
            f=f+1;
        end
       %% If the learning fails for MaxTrials times in this time domain, reinitialize the network
        if(f>MaxTrials) 
            [ANN,ANN_u,ANN_x,CNN,CNN_x] = createNetworks(agent);
            f=1;
        end
    end
end
if flag1==1
    disp(['Training complete']);
    break
end
end
toc % Stop timing the execution
JL=sum(J') % Calculate the sum of the performance metric J

figure % Plot the learning convergence condition verification
plot(tau:tau:3600*tau,Theorem4_1_robot{1},tau:tau:3600*tau,Theorem4_2_robot{1},'linewidth',3);
hold on
xlabel('Time (s)','FontName', 'Times New Roman');
yline(1,'-','Threshold','FontName','Times New Roman','fontsize',20,'linewidth',3);
 title('Learning convergence condition verification','fontsize',20,'Interpreter','latex','FontName', 'Times New Roman')
 axis([0,70,-0.2,1.2]);
 set(gca,'FontName','Times New Roman','FontSize',20,'LineWidth',1.5);

 J_2d=[];
 CosT_d{2}=J_1d(1);
for iter=1:180
    CosT_d{2}=CosT_d{2}*0.995^(iter^0.6);
    J_2d=[J_2d,CosT_d{2}];
end

figure % Plot the stability verification
plot(tau:tau:180*tau,J_1d(end-179:end),tau:tau:180*tau,J_2d(end-179:end),tau:tau:180*tau,J_3d(end-179:end),tau:tau:180*tau,J_1(end-179:end),'linewidth',3);
xlabel('Time (s)','FontName', 'Times New Roman');
ylabel('Cost $\bar V$','FontName', 'Times New Roman','Interpreter','latex');
title('Stability verification','fontsize',16,'Interpreter','latex','FontName', 'Times New Roman')
set(gca,'FontName','Times New Roman','FontSize',18,'LineWidth',1.5);
 axis([0,6,0,100]);
