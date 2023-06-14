function NN=NNProcess1(NN,input)
NN.NetworkIn=input;
NN.NetworkOut=NN.W1*tanh(NN.NetworkIn)+NN.B1;
