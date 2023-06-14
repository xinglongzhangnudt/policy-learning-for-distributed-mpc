function [Tau_42b]=T_42b(R,ANN1_LR,ANN_x1_LR,ANN_u1_LR,ANN1eta,dB_x1,dB_u1)
lamada_max=max(diag(R));
Tau_42b=lamada_max*(ANN1_LR*norm(ANN1eta,2)^2+ANN_x1_LR*norm(dB_x1,2)^2+ANN_u1_LR*norm(dB_u1,2)^2);