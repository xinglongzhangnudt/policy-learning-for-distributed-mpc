clear all;close all;
B=[];A=[];
for i=-10:0.01:20
    [B_x]=B_new([i;0],[10;0]);
    B=[B,B_x];
end
plot(B);
for i=-2:0.01:2
    [A_x]=A_new([1;1],[-1;-1],[i;i]);
    A=[A,A_x(1)];
end
plot(A);

