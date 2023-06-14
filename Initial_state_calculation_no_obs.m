function [State,R_State,V,NIError] =Initial_state_calculation_no_obs();
State{1}=[0;1.5;0;1.5];% no collision
State{2}=[0;0;0;1.5];
R_State=[0;1.5;0;1.5];
Thita1=State{1}(3);Thita2=State{2}(3);
T1 = calc_T(Thita1); 
T2 = calc_T(Thita2);
V=diag([1,1,0,0]);
NIError{1}=T1*((R_State-State{1})+(V*(State{2}-State{1})+[0;1;0;0]));
NIError{2}=T2*((R_State-State{2}+[0;-1;0;0])+(V*(State{1}-State{2})+[0;-1;0;0]));