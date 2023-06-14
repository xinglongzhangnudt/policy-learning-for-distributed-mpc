function [Tau]=T_1(z,z_next,F,rate)
Tau=rate*(norm(z,2)^2-2*norm(z'*z_next,2)*norm(F,2)+norm(z_next,2)^2+norm(F,2)^2);