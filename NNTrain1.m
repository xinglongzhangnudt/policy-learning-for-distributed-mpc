function NN=NNTrain1(NN,NNError)
Delta2=NNError;
dB1=Delta2;
dW1=Delta2*tanh(NN.NetworkIn)';
NN.W1=NN.W1-NN.LR*dW1;
NN.B1=NN.B1-NN.LR*dB1;