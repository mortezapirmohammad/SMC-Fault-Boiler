%% System definition %%


A = [3.7451e-15 7.6548e-6 0 0;-4.0887e-6 -6.5527e-2 0 0;...
    2.3773e-6 5.9026e-4 -0.1426 0;-8.1593e-14 -5.5355e-2 18.216 0.08333];eig(A)
B = [0.0015 -0.0015 -6.9678e-12;-5.9548e-5 -9.0316e-5 5.9647e-11;...
    3.4622e-5 5.2512e-5 3.2492e-11;-0.0167 0.0239 -1.0733e-9];
C = [0.05 -0.0484 6.7129 0.05;0 1 0 0];
D = [0 0 0 ; 0 0 0];

F = ones(4,1);
x0 = zeros(4,1)+1;
%% Reference model definition %%
A_m = [-0.01 0 0;0 -0.02 0;0 0 -0.03];
B_m = [1 0 0 ; 0 1 0 ; 0 0 1];
C_m = [0 1 0;1 0 0];
D_m = [0 0 0;0 0 0];

%% Controller design %%
Bound = 1e-3;
% syms g1 g2 g3 real
% Gamma = diag([g1,g2,g3])
Gamma = 1*diag([1,1,1]);
% 1 + max(eig(K_D*B*Gamma))

% syms g1 g2 g3 g4 g5 g6 g7 g8 g9 g10 g11 g12 real
% g1 = 1;
% g3 = 1;
% % g7 = 0;
% g7 = .5;
% g2 = 0;
% % g8 = 0;
% g8 = 1;
% g11 = 20;
% g9 = 0;
% g12 = -1;
% g10 = -1/625*20;
% G = [g1 g2 g3; 1 0 0;g7 g8 g9;g10 g11 g12];
% 
% % H = [g1 g2 g3; g4 g5 g6;g7 g8 g9];
% H = [67.4158930774610,486.813163566945,77.4940131414010;59.2368783438983,481.171184580240,56.7358936545988;324.564943,121.00527,163.367276];
% % C*G-C_m
% % M = [g1 g2 g3;g4 g5 g6;g7 g8 g9];
% M = [-92.64102639464,471.24710700615,318.45498822513;0,4656.64,2497.35370656213;143.290,11754404113.0869,69.60823];
% B*M-G*B_m
% Y = A*G  -G*A_m ;
% Y = [Y(1:2,1:3);Y(4,:)];
% X = B*H;X =X(1:3,1:3);
% H = [B(1:2,1:3);B(4,:)]^-1*Y;
% H = eye(3);
% K_P = [1.3731 0 0 1.3731;0 1.3731 0 0;0 0 1.3731 0];
% K_I = [2.6355 0 0 2.6355;0 2.6355 0 0;0 0 2.6355 0];
H = [-0.00264411452837401,0.276368234354978,0.00485669922298695;-0.00212093673792562,0.126828183853013,0.00297639139790861;-11263.6759461726,221638.898137783,1282.33499200163];
G = [0.968,-114.258,-1;1,0,0;0,1,0;0,0,1];
M = [0.322180266848393,-23.9048793908242,-0.198387888997022;0.273413353570640,-16.9472760226837,-0.136907357417107;1077571.96327408,-5503793.87571974,-55561.6496148715];
K_D = [0.0691 0 0 0.0691;0 0.0691 0 0;0 0 0.0691 0];
% K_S = [1.4800 0 0; 0 1.4800 0;0 0 1.4800];
K_I=2*[eye(3),[1;0;0]]; % n_u n_x
% K_D=0.2*[eye(3),[1;0;0]]; % n_u n_x
K_P=.1*[eye(3),[1;0;0]]; % n_u n_x
K_S=0.01*eye(3); % n_u n_u
gamma = 0.0013;
alpha_0 = 8.0314;
alpha_1 = 1.4713;


%% Running simulation #1
dB = 0;
dC = 0;
um_factor = 0;
fault =0;
sim simulation.slx


% Plot

figure(1)
subplot(1,2,1)
plot(y1)
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 1')
legend('y_1','y_m1')
xlabel('Time (s)')
subplot(1,2,2)
plot(ey1)
title('Output error 1')
grid
xlabel('Time (s)')

figure(2)
subplot(1,2,1)
plot(y2)
% xlim([80 180])
% ylim([.2 1.8])
grid
xlabel('Time (s)')
title('Bolier and Reference System Output 2')
legend('y_2','y_m2')
subplot(1,2,2)
plot(ey2)
title('Output error 2')
xlabel('Time (s)')
grid

figure(3)
subplot(3,1,1)
plot(xm1)
title('States of the reference model')
xlabel('')
grid
subplot(3,1,2)
plot(xm2)
grid
title('')
xlabel('')
subplot(3,1,3)
plot(xm3)
title('')
xlabel('Time (s)')
grid

figure(4)
subplot(4,1,1)
plot(x1)
xlabel('')
grid
title('States of the linearized model')
subplot(4,1,2)
plot(x2)
xlabel('')
title('')
grid
subplot(4,1,3)
plot(x3)
xlabel('')
title('')
grid
subplot(4,1,4)
plot(x4)
xlabel('Time (s)')
grid
title('')


figure(5)
subplot(4,1,1)
plot(z1)
xlabel('')
grid
title('Auxilary System States')
subplot(4,1,2)
plot(z2)
xlabel('')
title('')
grid
subplot(4,1,3)
plot(z3)
xlabel('')
title('')
grid
subplot(4,1,4)
plot(z4)
xlabel('Time (s)')
grid
title('')

%% Running simulation #2
um_factor = 1;
fault =0;
dB = 1e-4*[1 2 0;0 0 0; 1 -1 0 ;20 -20 0];
dC = 0.0002*[1 0 0 0;0 0 0 0];
sim simulation.slx

figure(6)
subplot(2,2,1)
plot(y1)
xlabel('')
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 1 (in presence of fault)')
legend('y_1','y_m1')
subplot(2,2,2)
plot(ey1)
title('Output error 1')
grid
xlabel('')

subplot(2,2,3)
plot(y2)
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 2 (in presence of fault)')
legend('y_2','y_m2')
xlabel('Time (s)')

subplot(2,2,4)
plot(ey2)
title('Output error 2')
xlabel('Time (s)')
grid

%% Running simulation #3
um_factor = 1;
fault =0;
dB = 1e-4*[1 2 0;0 0 0; 1 -1 0 ;20 -20 0]*0;
dC = 0.0002*[1 0 0 0;0 0 0 0];
sim simulation.slx

figure(7)
subplot(2,2,1)
plot(y1)
xlabel('')
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 1 (in presence of fault)')
legend('y_1','y_m1')
subplot(2,2,2)
plot(ey1)
title('Output error 1')
grid
xlabel('')

subplot(2,2,3)
plot(y2)
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 2 (in presence of fault)')
legend('y_2','y_m2')
xlabel('Time (s)')

subplot(2,2,4)
plot(ey2)
title('Output error 2')
xlabel('Time (s)')
grid

figure(8)
subplot(2,2,1)
plot(y1)
xlabel('')
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 1 (in presence of fault)')
legend('y_1','y_m1')
subplot(2,2,2)
plot(ey1)
title('Output error 1')
grid
xlabel('')

subplot(2,2,3)
plot(y2)
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 2 (in presence of fault)')
legend('y_2','y_m2')
xlabel('Time (s)')

subplot(2,2,4)
plot(ey2)
title('Output error 2')
xlabel('Time (s)')
grid


%% Running simulation #4
um_factor = 1;
fault =0;
dB = 1e-4*[1 2 0;0 0 0; 1 -1 0 ;20 -20 0]*2;
dC = 0.0002*[1 0 0 0;0 0 0 0];
sim simulation.slx

figure(9)
subplot(2,2,1)
plot(y1)
xlabel('')
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 1 (in presence of fault)')
legend('y_1','y_m1')
subplot(2,2,2)
plot(ey1)
title('Output error 1')
grid
xlabel('')

subplot(2,2,3)
plot(y2)
% xlim([80 180])
% ylim([.2 1.8])
grid
title('Bolier and Reference System Output 2 (in presence of fault)')
legend('y_2','y_m2')
xlabel('Time (s)')

subplot(2,2,4)
plot(ey2)
title('Output error 2')
xlabel('Time (s)')
grid

