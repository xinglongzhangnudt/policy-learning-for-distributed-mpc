clear
load distributedd_cost0623v0.mat %v0,...v5 for different initial conditions
Jd=J;
Ud=U;
Ed=E;
load centralized_cost0623v0.mat %v0,...v5 for different initial conditions
figure
subplot(2,1,1)
plot(tau:tau:tau*size(Ud,2),Ud(1,:),'b-','LineWidth',3)
hold on 
plot(tau:tau:tau*size(Ud,2),U(1,:),'-.','Color',[0.85,0.33,0.1],'LineWidth',3)
axis([0,tau*200,-0.2,4])
title('$a_v$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);

subplot(2,1,2)
plot(tau:tau:tau*size(Ud,2),Ud(2,:),'b-','LineWidth',3)
hold on
plot(tau:tau:tau*size(Ud,2),U(2,:),'r-.','LineWidth',3)
axis([0,tau*200,-0.2,1])
title('$a_w$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);
 xlabel('Time (s)','FontName', 'Times New Roman');

figure
subplot(4,2,1)
plot(tau:tau:tau*size(Ed,2),Ed(1,:),'b-',tau:tau:tau*size(Ed,2),E(1,:),'r-.','LineWidth',3)
axis([0,tau*500,0,0.3])
title('$e_{x}$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);
subplot(4,2,2)
plot(tau:tau:tau*size(Ed,2),Ed(2,:),'b-',tau:tau:tau*size(Ed,2),E(2,:),'r-.','LineWidth',3)
axis([0,tau*500,-0.05,0.1])
title('$e_{y}$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);
 legend('Ours', 'Centralized','FontName','Times New Roman')
% subplot(5,2,3)
% plot(tau:tau:tau*size(Ed,2),Ed(3,:),'-',tau:tau:tau*size(Ed,2),E(3,:),'-.','LineWidth',3)
subplot(4,2,3)
plot(tau:tau:tau*size(Ed,2),Ed(4,:),'b-',tau:tau:tau*size(Ed,2),E(4,:),'r-.','LineWidth',3)
axis([0,tau*500,0,0.05])
xlabel('Time (s)','FontName', 'Times New Roman');
title('$e_{\theta}$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);
subplot(4,2,4)
plot(tau:tau:tau*size(Ed,2),Ed(4,:),'b-',tau:tau:tau*size(Ed,2),E(4,:),'r-.','LineWidth',3)
axis([0,tau*500,0,0.05])
xlabel('Time (s)','FontName', 'Times New Roman');
title('$e_w$','FontName', 'Times New Roman','Interpreter','latex');
 set(gca,'FontName','Times New Roman','FontSize',16,'LineWidth',1.5);
% figure
% plot(1:150,Jd(:,end),'b',1:150,J(:,end),'r')
figure
plot(1:120,((Jd(1:120,end)-J(1:120,end))/1600),'LineWidth',3, ...
            'LineStyle','-','Color','blue')
hold on
plot(1:6:120,((Jd(1:6:120,end)-J(1:6:120,end))/1600),'.','LineWidth',5, ...
            'MarkerSize',30,...
    'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6])
grid;
xlabel('Learning Episodes','FontName', 'Times New Roman');
ylabel('$\Delta J=J_{\rm our}-J_{c}$','FontName', 'Times New Roman','Interpreter','latex');
 title('Cost gap to the centralized solution','fontsize',18,'Interpreter','latex','FontName', 'Times New Roman')
 set(gca,'FontName','Times New Roman','FontSize',18,'LineWidth',1.5);
 