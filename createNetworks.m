function [ANN, ANN_u,ANN_x,CNN,CNN_x,state_bound] = createNetworks(agent) 
   load safe_actor_critic.mat;
for i = 1:agent 
    ANN{i} = CreateANN1(8,2); 
    ANN_u{i} = CreateANN_x(2,2); 
    ANN_x{i} = CreateANN_x(2,2); 
    CNN{i} = CreateCNN1(8,8); 
    CNN_x{i} = CreateCNN_x(2,2); 
    ANN_x{1}=ANN_x1;ANN_x{2}=ANN_x2;
    state_bound{1}=[2;2;2;2]; 
    state_bound{2}=[2;2;2;2]; 
end
end