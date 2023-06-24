function NN=ANN_xTrain(NN,NNError)
Delta2=NNError;
dB1=Delta2;
dW1=Delta2*NN.NetworkIn';
NN.W1=NN.W1-NN.LR*dW1;


