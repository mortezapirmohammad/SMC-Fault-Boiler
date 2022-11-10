 syms t1 t2 t3 t4 t5 t6 t7 t8 t9 t10 t11 t12 t13 t14 t15 t16 real
 
 P = [t1 t2 t3 t4; t5 t6 t7 t8; t3 t7 t11 t12 ;t4 t8 t15 t16];
 
 x = D'-C*P
 s = solve(x(2,:));
 t4 = s.t4;
 t8 = s.t8;
 t15 = s.t15;
 t16 = s.t16;
 t3 = 0;t7 = 0;t11 = 1;t12 = 0;t4 = 0;t8 = 0;t15 = 0;t16 = 1;
 t1 = 1; t6 = 1;t2 = -1;t5 = 0;
 P = [t1 t2 t3 t4; t5 t6 t7 t8; t3 t7 t11 t12 ;t4 t8 t15 t16];
 
 x = D'-C*P
 s = solve(x(1,:));
 t3 = s.t3;
 t7 = s.t7;
 t11 = s.t11;
 t12 = s.t12;
 P = double([t1 t2 t3 t4; t5 t6 t7 t8; t3 t7 t11 t12 ;t4 t8 t15 t16]);
 
 x = D'-C*P
 eig(P)
P = eye(4);
P1 = P(1:n-p,1:n-p);P2 = P(1:n-p,n-p+1:end);P3 = P2';P4 = P(n-p+1:end,n-p+1:end);

S = eye(2);