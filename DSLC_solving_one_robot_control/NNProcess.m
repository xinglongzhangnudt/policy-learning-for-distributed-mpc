function NN=NNProcess(NN,input)

NN.NetworkIn=input;
NN.HiddenOut=logsig(NN.W1*NN.NetworkIn+NN.B1);
NN.NetworkOut=NN.W2*NN.HiddenOut+NN.B2;
NN.Jacobian=NN.W2*((NN.HiddenOut).*(1-NN.HiddenOut)*ones(1,NN.InDim).*NN.W1);