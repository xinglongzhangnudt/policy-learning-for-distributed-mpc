function [Tau_42a]=T_42a(CNN1eta,CNN1eta_f,F1,CNN1_LR,dB_x1,dB_x1f,CNN_x1_LR)
Tau_CNN1=CNN1_LR*(norm(CNN1eta,2)^2-2*norm(CNN1eta'*CNN1eta_f,2)*norm(F1,2)+norm(CNN1eta_f,2)^2+norm(F1,2)^2);
Tau_B1=CNN_x1_LR*(norm(dB_x1,2)^2-2*norm(dB_x1'*dB_x1f,2)*norm(F1,2)+norm(dB_x1f,2)^2+norm(F1,2)^2);
Tau_42a=Tau_CNN1+Tau_B1;