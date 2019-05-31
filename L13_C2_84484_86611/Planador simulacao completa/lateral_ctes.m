deg = pi/180;
kt = 1.852/3.6;
g = 9.81;

% L13 : flight condition : 2
h =500 ; % m
M =0.10 ;
aa0 = -1.95*deg ; %alpha
gg0 = -2*deg ;    %gama
tt0 = aa0+gg0;   %picada
u0 =68.5*kt ; % m/s (velocidade horizontal)
w0= u0*tan(aa0); % m/s (velocidade vertical)
flaps =0*deg ;
de0 =0.00*deg ;
da0 =0.00*deg ;
dr0 =0.00*deg ;
%demax =[+28 -21] ;
damax =17*deg ;
drmax =23*deg ;
flapmax =40*deg ;
spmax =60*deg ;

% inertial data :
m =463 ; % kg
Ix =1627 ; % kg.m^2
Iy =1220 ; % kg.m^2
Iz =5423 ; % kg.m^2
Ixz =0 ; % kg.m^2

%wing data : 
S =19.14 ; % m^2
b =16.180 ; % m
c =0.779 ; % m
aamax =11.43*deg ;

%derivatives ( no units or SI units ):
ybb = -0.4937;
lbb = -16.1786;
nbb = 0.1537;
yp = 0.0223;
lp = -3.2601;
np = 0.0697;
yr = 0.0873;
lr = -3.5764;
nr = -0.2288;

Lda = -23.868;
Nda = 0.449;
Ydr = -0.204;
Ldr = -2.367;
Ndr = -2.859;

A_lat = [ybb yp+w0/u0 yr-1 g*cos(tt0)/u0 0; lbb lp lr 0 0; nbb np nr 0 0; 0 1 tan(tt0) 0 0; 0 0 1/cos(tt0) 0 0];
B_lat = [0 Ydr ; Lda Ldr ; Nda Ndr ; 0 0 ; 0 0];
C_lat = eye(5);
D_lat = [0 0; 0 0; 0 0; 0 0; 0 0];

sys_lat  = ss(A_lat,B_lat,C_lat,D_lat);

Q_lat = diag([15*deg*0.2, 0.1, 0.1, 30*deg*0.2, 10*deg].^(-2));
R_lat = diag([17*deg*0.2, 23*deg*0.2].^(-2));
k_lat = lqr(sys_lat,Q_lat,R_lat);

Cpr_lat  = C_lat(2:1:3,:);
Cbetaphi_lat = C_lat(1:3:4,:);
Cpsi_lat = C_lat(5,:);
Kpr_lat  = k_lat(:,2:1:3);
Kbetaphi_lat = k_lat(:,1:3:4);
Kpsi_lat = g/u0;

Cc_lat = C_lat(2:1:4,:);
Cy_lat = C_lat(1:4:5,:);
Kc_lat = k_lat(:,2:1:4);
Ky_lat = k_lat(:,1:4:5);
k_g_lat = k_lat(:,5);

%sys_servo = feedback( (feedback(sys,Kc*Cc))*Ky , Cy );