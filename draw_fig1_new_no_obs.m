function [NIError,State,R_State]=draw_fig1_new_no_obs(ANN,ANN_x,ANN_u,ConHor_len,state_bound,NIError,State,R_State,Obstacle,k,Iterations_num)
% This function performs the iteration and drawing operation for given parameters.
% Inputs: ANN, ANN_x, ANN_u (networks for computation)
%         ConHor_len (control horizon length)
%         state_bound (state bounds)
%         NIError (initial error)
%         State (initial states)
%         R_State (desired state)
%         Obstacle (Obstacle positions)
%         k (current iteration number)
%         Iterations_num (total number of iterations)

% Define some global and persistent variables used in the function
global tau iter;
persistent E1 U1 X1  E2 X2  PresentState PE Present_x 

% Initialization for the first iteration
if(k==1)
    % Initialize arrays for Errors, Controls and States for the first iteration
    E1=zeros(4,Iterations_num);
    E2=zeros(4,Iterations_num);
    U1=zeros(2,Iterations_num);
    U2=zeros(2,Iterations_num);
    X1=zeros(4,Iterations_num);
    X2=zeros(4,Iterations_num);
    Y=zeros(3,Iterations_num);
    PresentState{1}=[0;0.9;0;1];
    PresentState{2}=[0;-0.1;0;1];
    PE=NIError;
    Present_x=[0;1;0;1];
end
% Initialize some variables for the while loop
V=diag([1,1,0,0]);
PresentError=NIError;
timesteps=0;  
PresentState=State;
Present_x=R_State;
Umax1=[5;5];Umin1=[-5;-5];
Umax2=[5;5];Umin2=[-5;-5];

% Loop until the desired control horizon length is reached
while(timesteps<ConHor_len)
    % Normalize the current error with respect to the state bounds
    c_input{1}=PresentError{1}./state_bound{1};        
    c_input{2}=PresentError{2}./state_bound{2};  
    % Update the neural networks based on the current input
    ANN{1}=NNProcess1(ANN{1},[c_input{1};c_input{2}]);       
    ANN{2}=NNProcess1(ANN{2},[c_input{2};c_input{1}]); 
    % Calculate the barrier function based on current input
    [dB_u1]=Barriera(Umax1,Umin1,ANN{1}.NetworkOut);
    [dB_u2]=Barriera(Umax2,Umin2,ANN{2}.NetworkOut);
    [dB_x1]=Barrierx(PresentState{1},Obstacle);
    [dB_x2]=Barrierx(PresentState{2},Obstacle);
    % Set dB_x1 and dB_x2 to zero when PresentState{1} is larger than 30
    if(PresentState{1}>30)
      dB_x1=[0;0];dB_x2=[0;0];
    end
    ANN_u{1}=ANN_xProcess(ANN_u{1},dB_u1);
    ANN_u{2}=ANN_xProcess(ANN_u{2},dB_u2);
    ANN_x{1}=ANN_xProcess(ANN_x{1},dB_x1);
    ANN_x{2}=ANN_xProcess(ANN_x{2},dB_x2);
    u1=ANN{1}.NetworkOut+ANN_u{1}.NetworkOut+0*ANN_x{1}.NetworkOut;
    u2=ANN{2}.NetworkOut+ANN_u{2}.NetworkOut+0*ANN_x{2}.NetworkOut;
    
    % The FutureStates are obtained by applying the calculated controls to the current states.
    FutureState{1}=robot(PresentState{1},u1);
    FutureState{2}=robot(PresentState{2},u2);
   
    Future_x=desired_position(Present_x);   % Future_x is the desired position for the next time step
    Thita1=FutureState{1}(3);
    Thita2=FutureState{2}(3);
    T1=calc_T(Thita1); 
    T2 =calc_T(Thita2); 
    
    % Calculate future error based on transformations, current and future states.
    FE{1}=T1*((Future_x-FutureState{1})+(V*(FutureState{2}-FutureState{1})+[0;1;0;0]));
    FE{2}=T2*((Future_x-FutureState{2}+[0;-1;0;0])+(V*(FutureState{1}-FutureState{2})+[0;-1;0;0]));
  
    FutureError=FE;
    E1(:,k+timesteps)=PE{1};
    E2(:,k+timesteps)=PE{2};
    X1(:,k+timesteps)=PresentState{1};
    X2(:,k+timesteps)=PresentState{2};
    U1(:,k+timesteps)=u1;
    U2(:,k+timesteps)=u2;
    PresentError=FutureError;
    PresentState=FutureState;
    Present_x=Future_x;
    PE=FE;
    timesteps=timesteps+1;
end
% Return the final Error, State, and R_State
NIError=PE;
State=PresentState;
R_State=Present_x;

% Plotting section
if(k+timesteps-1>=Iterations_num)
% Uncomment the following lines for visualizing the simulation
% The first figure shows state errors
% The second figure shows the trajectory of the robots
% The third figure shows control inputs

%     figure;
%     subplot(4,1,1);
%     plot(E1(1,:));
%     subplot(4,1,2);
%     plot(E1(2,:));
%     subplot(4,1,3);
%     plot(E1(3,:));
%     subplot(4,1,4);
%     plot(E1(4,:));
%     title('State error');
%% plot path
%     figure;
%     plot(X1(1,:),X1(2,:),'b');
%     hold on;
%     plot(X2(1,:),X2(2,:),'r');
%     hold on;
%     %plot(Obstacle(1),Obstacle(2),'ok','MarkerFaceColor','k');
%     rectangle('position',[Obstacle(1)-0.2,Obstacle(2)-0.2,0.2*2,0.2*2],'curvature',[1,1],'edgecolor','[0,0,0.5]','linestyle','-','facecolor','b');
%     title('Trajectory');
    %%
%     figure;
%     subplot(2,1,1);
%     plot(U1(1,:));
%     subplot(2,1,2);
%     plot(U1(2,:));
    %title('Control input');
end