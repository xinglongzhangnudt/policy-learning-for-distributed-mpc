function CNN=CreateCNN_x(x_dim,y_dim)
CNN.InDim=x_dim;            
CNN.OutDim=y_dim;                       
CNN.W1=0.1*(rand(CNN.OutDim,CNN.InDim)-0.5);   
CNN.B1=0.1*(rand(CNN.OutDim,1)-0.5);       
CNN.LR=0.00001;

    

