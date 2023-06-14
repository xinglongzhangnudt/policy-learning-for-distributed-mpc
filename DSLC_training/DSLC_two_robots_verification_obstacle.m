clear all;close all;
global tau iter Bx1 Bx2;
load P_VALUE_two.mat % Load the given .mat files
load K_two.mat
%% Initialization
% Initialize constants, including control horizon, prediction horizon, barrier centers, obstacles, etc.
[tau,Gamma,Iterations_num,MaxTrials,ConHor_len,PreHor_len,center_barrier,Obstacle,mu,Q,R,run,scale,Umax,Umin] = set_constants();
agent=2; % Set agent count to 2
[ANN,ANN_u,ANN_x,CNN,CNN_x,state_bound] = createNetworks(agent); % Create network structures for each agent
[State0,R_State0,V,NIError0] =Initial_state_calculation; % Compute the initial state
J=zeros(run,Iterations_num);J_1=[];% Preallocate memory for performance measurement
flag1=0; % Flag used to indicate if an obstacle is too close
tic % Start timing the execution
%% Main loop
for iter=1:run
       NIError=NIError0;
       State=State0;
       R_State=R_State0;
   disp(['run=',num2str(iter)]);  % Display the current run number
   Theorem4_1_roboT{1}=[];Theorem4_2_roboT{1}=[];
   Theorem4_1_roboT{2}=[];Theorem4_2_roboT{2}=[];
for k=1:ConHor_len:Iterations_num   % Loop over control horizon
    disp(['iteration=',num2str(k)]); % Display the current iteration number
    RealError=NIError;
    RealState=State;
    Present_rx=R_State;
    f=1;CosT{1}=0;
    %% Determine whether the learning is successful in the current prediction time domain
    while(f>=1)
        PresentError=RealError;
        PresentState=RealState;
        Present_x=Present_rx;
        timesteps=0;  
        j=0;
        while(timesteps<PreHor_len)
            for i=1:agent
            c_input{i}=PresentError{i}./state_bound{i};
            end
            ANN{1}=NNProcess1(ANN{1},[c_input{1};c_input{2}]);
            ANN{2}=NNProcess1(ANN{2},[c_input{2};c_input{1}]); %W_a*sigma_a
            for i=1:agent
            ANN_eta{i}=tanh(ANN{i}.NetworkIn);
            end

            distance = sqrt((PresentState{1}(1)-Obstacle(1))^2+(PresentState{1}(2)-Obstacle(2))^2);
            if distance<0.3%;%0.2+0.03
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
 %% Update State
            Future_x=desired_position(Present_x);
           [MSJ1,MCJ1,MSJ12]=sys_process12(PresentError{1},u{1},PresentState{1},PresentState{2});
           [MSJ2,MCJ2,MSJ21]=sys_process21(PresentError{2},u{2},PresentState{1},PresentState{2});
           
           for i=1:agent
           FutureState{i}=robot(PresentState{i},u{i});
           Thita{i}=FutureState{i}(3);
           T{i} = calc_T(Thita{i}); 
           end
           
            FutureError{1}=T{1}*((Future_x-FutureState{1})+(V*(FutureState{2}-FutureState{1})+[0;1;0;0]));
            FutureError{2}=T{2}*((Future_x-FutureState{2}+[0;-1;0;0])+(V*(FutureState{1}-FutureState{2})+[0;-1;0;0]));
 %% CNN output  
            for i=1:agent
            f_input{i}=FutureError{i}./state_bound{i};
            end
           
            dR_dZ1=2*Q*[PresentError{1};PresentError{2}]+mu*[-0.5.*T{1}^-1*[dB_x{1};0;0];0;0;0;0];         
            dR_dZ2=2*Q*[PresentError{2};PresentError{1}]+mu*[-0.5.*T{1}^-1*[dB_x{2};0;0];0;0;0;0]; 
            cosT{1}=2*[PresentError{1};PresentError{2}]'*Q*[PresentError{1};PresentError{2}]+mu*B_x{1}+mu*B_u{1}+u{1}'*R*u{1};            CosT{1}=CosT{1}+cosT{1};
           

                CNN{1}=NNProcess1(CNN{1},[f_input{1};f_input{2}]);
                CNN{2}=NNProcess1(CNN{2},[f_input{2};f_input{1}]);      
                for i=1:agent
                CNN_eta_f{i}=tanh(CNN{i}.NetworkIn);
                [dB_x{i}]=Barrierx(FutureState{i},Obstacle);
                [B_x{i}]=B_new(FutureState{i},Obstacle);
                CNN_x{i}=CNN_xProcess(CNN_x{i},dB_x{i});%barrier_x
                FutureLambda{i}=CNN{i}.NetworkOut+[CNN_x{i}.NetworkOut;0;0;0;0;0;0];  
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
            CNNTargeT{1}=dR_dZ1+Gamma*[MSJ1',MSJ21';MSJ12',MSJ2']*FutureLambda{1}; % estimate of lambda(t)   
            CNNTargeT{2}=dR_dZ2+Gamma*[MSJ2',MSJ12;MSJ21',MSJ1']*FutureLambda{2};
            
            for i=1:agent 
            CNNError{i}=PresentLambda{i}-CNNTargeT{i};
            end
        
            F1=[MSJ1',MSJ21';MSJ12',MSJ2'];
            
            [Tau_42a_roboT{1}]=T_42a(CNN_eta{1},CNN_eta_f{1},F1,CNN{1}.LR,B_x{1},B_x{1},CNN_x{1}.LR);
            Theorem4_1_roboT{1}=[Theorem4_1_roboT{1},Tau_42a_roboT{1}];
            
            [Tau_42b_roboT{1}]=T_42b(R,ANN{1}.LR,ANN_x{1}.LR,ANN_u{1}.LR,ANN_eta{1},dB_x{1},dB_u{1});
            Theorem4_2_roboT{1}=[Theorem4_2_roboT{1},Tau_42b_roboT{1}];
            
            [Tau_42a_roboT{2}]=T_42a(CNN_eta{2},CNN_eta_f{2},F1,CNN{2}.LR,B_x{2},B_x{2},CNN_x{2}.LR);
            Theorem4_1_roboT{2}=[Theorem4_1_roboT{2},Tau_42a_roboT{2}];
            
            [Tau_42b_roboT{2}]=T_42b(R,ANN{2}.LR,ANN_x{2}.LR,ANN_u{2}.LR,ANN_eta{2},dB_x{2},dB_u{2});
            Theorem4_2_roboT{2}=[Theorem4_2_roboT{2},Tau_42b_roboT{2}];
            
            for i=1:agent 
            CNN{i}=NNTrain1(CNN{i},CNNError{i});   % train CNN
            CNN_x{i}=CNN_xTrain(CNN_x{i},CNNError{i}(1:2));
            end
            PresentState=FutureState;
            PresentError=FutureError;
            Present_x=Future_x;
            timesteps=timesteps+1; 
            if timesteps==2
            j=j+[PresentError{1};PresentError{2}]'*Q*[PresentError{1};PresentError{2}]+u{1}'*R*u{1}+[PresentError{2};PresentError{1}]'*Q*[PresentError{2};PresentError{1}]+u{2}'*R*u{2};
            end
            end
        J(iter,k)=j;        J_1=[J_1,CosT{1}]; 
       %% Determine whether learning is successful whether learning is successful
        if(timesteps==PreHor_len)  %% success
            [NIError,State,R_State]=draw_fig1_new(ANN,ANN_x,ANN_u,ConHor_len,state_bound,NIError,State,R_State,Obstacle,k,Iterations_num);
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
% if flag1==1  % If an obstacle is too close, training is complete
%     disp(['Training complete']);
%     break
% end
end
toc % Stop timing the execution
JL=sum(J') % Calculate the sum of the performance metric J


figure % Plot the learning convergence condition verification
plot(tau:tau:3600*tau,Theorem4_1_roboT{1},tau:tau:3600*tau,Theorem4_2_roboT{1},'linewidth',3);
hold on
xlabel('Time (s)','FontName', 'Times New Roman');
yline(1,'-','Threshold','FontName','Times New Roman','fontsize',20,'linewidth',3);
 title('Learning convergence condition verification','fontsize',20,'Interpreter','latex','FontName', 'Times New Roman')
 axis([0,70,-0.2,1.2])
 set(gca,'FontName','Times New Roman','FontSize',20,'LineWidth',1.5);

