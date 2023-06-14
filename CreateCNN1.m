function CNN=CreateCNN1(x_dim,y_dim)
CNN.HiddenUnitNum=5;    
CNN.InDim=x_dim;            
CNN.OutDim=y_dim;            
CNN.LR=0.4;           
CNN.W1=0.1*(rand(CNN.OutDim,CNN.InDim)-0.5); 
CNN.B1=0.1*(rand(CNN.OutDim,1)-0.5);     
