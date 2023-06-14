function NN=NNTrain(NN,NNError)

Delta2=NNError;
Delta1=NN.W2'*Delta2.*(NN.HiddenOut).*(1-NN.HiddenOut);

dW2=Delta2*NN.HiddenOut';
dB2=Delta2;
dW1=Delta1*NN.NetworkIn';
dB1=Delta1;

NN.W1=NN.W1-NN.LR*dW1;
NN.B1=NN.B1-NN.LR*dB1;
NN.W2=NN.W2-NN.LR*dW2;
NN.B2=NN.B2-NN.LR*dB2;
