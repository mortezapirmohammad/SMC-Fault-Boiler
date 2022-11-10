clc
clear all
close all
warning off

%% System definition


A = [3.7451e-15 7.6548e-6 0 0;-4.0887e-6 -6.5527e-2 0 0;...
    2.3773e-6 5.9026e-4 -0.1426 0;-8.1593e-14 -5.5355e-2 18.216 0.08333];
B = [0.0015 -0.0015 -6.9678e-12;-5.9548e-5 -9.0316e-5 5.9647e-11;...
    3.4622e-5 5.2512e-5 3.2492e-11;-0.0167 0.0239 -1.0733e-9];
C = [0.05 -0.0484 6.7129 0.05;0 1 0 0];
dB = 1e-4*[1 2 0;0 0 0; 0 0 -1 ;20 -20 0]; % Uncertainty matrix
dC = 0.0002*[1 0 0 0;0 0 0 0]; % Uncertainty matrix
E = dB;

D = [0 0 0 20]'; % Actuator fault matrix
N = [0 1]'; % Sensor fault matrix

n = 4; % number of states (x)
m = 3; % number of control inputs (u)
p = 2; % number of system outputs (y)
psi_a = 1; % number of actuator faults (fa)
psi_s = 1; % number of sensor faults (fs)
sai = 1e-3;  % bound of uncertainty signal

C_h = C(1,:); % healthy output matrix
C_f = C(2,:); % faulty output matrix
Omega = 10; % filter gain

Ab = [A zeros(n,psi_s);Omega*C_f -Omega]; % A_bar
Bb = [B;zeros(psi_s,m)]; % B_bar
Cb = [C_h zeros(p-psi_s,psi_s);zeros(psi_s,n) eye(psi_s)]; % C_bar
Db = [D zeros(n,psi_s);zeros(psi_s,psi_a) Omega]; % D_bar
T = [[eye(3) zeros(3,2)];Cb]; % Transformation


Ab_p = T*Ab*T^-1; % A_bar_prime
Cb_p = Cb*T^-1; % C_bar_prime
Db_p = T*Db; % D_bar_prime

P_cal % mfile for calculating P

A1 = Ab_p(1:n-p,1:n-p);
A2 = Ab_p(1:n-p,n-p+1:end);
A3 = Ab_p(n-p+1:end,1:n-p);
A4 = Ab_p(n-p+1:end,n-p+1:end);
D1 = Db_p(1:n-p,:);
D2 = Db_p(n-p+1:end,:);
pa = 10; % bound of actuator fault
ps = 0.1; % bound of sensor fault
eta1 = 20;
P0 = eye(p); % design matrix
A0 = -100*eye(p); % design matrix
A0 = [zeros(3,2);A0]; % design matrix_bar
k = norm(D2)*pa + eta1; % sliding mode coefficient



%% Controller %%
A_m = [-0.01 0 0;0 -0.02 0;0 0 -0.03];
B_m = [1 0 0 ; 0 1 0 ; 0 0 1];
C_m = [0 1 0;1 0 0];
D_m = [0 0 0;0 0 0];
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
um_factor = 0;


%% Simultaneous fault reconstruction and FTC %%
%% without fault
f_a = 0;
f_s = 0;
FTC_factor = 0;
sim act_sens.slx
x1_healthy = x1;
x2_healthy = x2;
x3_healthy = x3;
x4_healthy = x4;




f_a = 10; % actuator fault amplitude
f_s = 0.1; % sensor fault amplitude
FTC_factor = 0; % fault tolerant control : No:0   Yes:1
sim act_sens.slx
x1_fault = x1;
x2_fault = x2;
x3_fault = x3;
x4_fault = x4;

%% Simultaneous faults with FTC
f_a = 10;
f_s = 0.1;
FTC_factor = 1;
sim act_sens.slx
x1_FTC = x1;
x2_FTC = x2;
x3_FTC = x3;
x4_FTC = x4;


%% Plot

% FTC implementation
figure(1)
subplot(4,1,1)
plot(x1_fault,'b')
hold on
plot(x1_FTC,'r')
hold on
plot(x1_healthy,'g')
legend('No FTC','plus FTC','No fault')
ylabel('x_1')
grid

subplot(4,1,2)
plot(x2_fault,'b')
hold on
plot(x2_FTC,'r')
hold on
plot(x2_healthy,'g')
legend('No FTC','plus FTC','No fault')
ylabel('x_2')
grid

subplot(4,1,3)
plot(x3_fault,'b')
hold on
plot(x3_FTC,'r')
hold on
plot(x3_healthy,'g')
legend('No FTC','plus FTC','No fault')
ylabel('x_3')
grid

subplot(4,1,4)
plot(x4_fault,'b')
hold on
plot(x4_FTC,'r')
hold on
plot(x4_healthy,'g')
legend('No FTC','plus FTC','No fault')
ylabel('x_4')
xlabel('Time(s)')
ylim([-1100,500])
grid

% Actuator fault reconstruction
figure(2)
plot(fa)
legend('Real signal','estimated signal')
xlabel('Time(s)')
title('Actuator fault reconstruction')
grid
ylim([-20,20])

% Sensor fault reconstruction
figure(3)
plot(fs)
legend('Real signal','estimated signal')
xlabel('Time(s)')
title('Sensor fault reconstruction')
grid
ylim([-0.5,0.5])

% Observer performance
figure(4)
subplot(4,1,1)
plot(X1)
legend('x_1','x_1hat')
grid
title('Observer performance')

subplot(4,1,2)
plot(X2)
legend('x_2','x_2hat')
grid
title('')

subplot(4,1,3)
plot(X3)
legend('x_3','x_3hat')
grid
title('')

subplot(4,1,4)
plot(X4)
legend('x_4','x_4hat')
grid
title('')