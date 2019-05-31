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
demax =[+28 -21] ;
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

xu = -0.0378;
xw = 0.1992;
zu = -0.5557; 
zw = -3.8432;
zwp = -0.0400; 
zq = -3.9547;
mu = 0.0000;
mw = -1.0955;
mq = -5.5580;
mwp = 0.0512;

ybb = -0.4937;
lbb = -16.1786;
nbb = 0.1537;
yp = 0.0223;
lp = -3.2601;
np = 0.0697;
yr = 0.0873;
lr = -3.5764;
nr = -0.2288;

xde = 0.000;
zde = -12.014;
mde = -15.309;
xdf = -1.070;
zdf = -3.505;
mdf = -0.112;
xdsp = -2.139;
zdsp = 0.000;
mdsp = 0.000;

Lda = -23.868;
Nda = 0.449;
Ydr = -0.204;
Ldr = -2.367;
Ndr = -2.859;

A = [ybb yp+w0/u0 yr-1 g*cos(tt0)/u0 ; lbb lp lr 0 ; nbb np nr 0 ; 0 1 tan(tt0) 0]; % 0 0 1/cos(tt0) 0 0
B = [0 Ydr ; Lda Ldr ; Nda Ndr ; 0 0];
C = eye(4);
D = [0 0; 0 0; 0 0; 0 0];

damp(A)
sys2 = ss(A, B(:,2), -[1 0 0 0], 0);
sys  = ss([[A [0;0;0;0]];[0 0 1/cos(tt0) 0 0]], [B; 0 0], eye(5), 0);
%rlocus(sys2)

%maximo da espiral: polo em log(2)/12 = 0.0578 => ganho k = 0.4303, com [k, poles] = rlocfind(sys2, 0.0578)
%isto resulta num damp(A-0.4303*B(:,2)) de 0.4592 para RH

% n = log(2)/12;
% [k, poles] = rlocfind(sys2, n);
%damp(feedback(sys2,k))

%ponto 2: rlocus ou damp(feedback(pid(-0.474136, 0, -0.474136*2.20675)*sys2,1))

p = [-3.55 -0.77-0.7856i -0.77+0.7856i -0.3];
K = place(A,B,p);
damp(A-B*K);

