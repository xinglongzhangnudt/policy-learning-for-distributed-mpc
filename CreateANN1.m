function ANN=CreateANN1(x_dim,u_dim)
ANN.HiddenUnitNum=5;     
ANN.InDim=x_dim;             
ANN.OutDim=u_dim;            
ANN.LR=0.2;               
ANN.W1=0.1*(rand(ANN.OutDim,ANN.InDim)-0.5);   
ANN.B1=0.1*(rand(ANN.OutDim,1)-0.5);          