function NN=ANN_xProcess(NN,input)
NN.NetworkIn=input;
NN.NetworkOut=NN.W1*NN.NetworkIn;
