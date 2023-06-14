function NN=CNN_xProcess(NN,input)
NN.NetworkIn=input;
NN.NetworkOut=NN.W1*NN.NetworkIn;
NN.Jacobian=[NN.W1];
