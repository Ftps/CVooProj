%constantes gerais (unidades não especificadas estão em SI)
deg=pi/180;
g=9.81;
kt=0.514444;

%condição inicial de voo
h0=3048; M0=0.10; aa0=-1.95*deg; gg0=-2*deg; u0=68.5*kt; flaps=0*deg;
de0=0.00*deg; da0=0.00*deg; dr0=0.00*deg;
tt0=gg0+aa0;
inicial=[u0, aa0, 0, tt0, h0];

demax=28*deg; demin=-21*deg; damax=17*deg; drmax=23*deg; flapmax=40*deg; spmax=60*deg;

%inertial data
m=463; Ix=1627; Iy=1220; Iz=5423; Ixz=0;

%wing data
S=19.14; b=16.180; c=0.779; aamax=11.43*deg;

%derivatives (no units or SI units)
xu=-0.0378; xw=0.1992; zu=-0.5557; zw=-3.8432; zwp=-0.0400; zq=-3.9547; mu=0.0000; mw=-1.0955; mq=-5.5580; mwp=0.0512;
ybb=-0.4937; lbb=-16.1786; nbb=0.1537; yp=0.0223; lp=-3.2601; np=0.0697; yr=0.0873; lr=-3.5764; nr=-0.2288;

xde=0.000; zde=-12.014; mde=-15.309; xdf=-1.070; zdf=-3.505; mdf=-0.112; xdsp=-2.139; zdsp=0.000; mdsp=0.000;

Lda=-23.868; Nda=0.449; Ydr=-0.204; Ldr=-2.367; Ndr=-2.859;

%---------------------------------------------------------------------------------------------------------------------
%matriz da dinâmica
A=[xu,        xw*u0,         -u0*tan(aa0), -g*cos(tt0),     0;
   zu/u0,     zw,             1,           -g/u0*sin(tt0),  0;
   mu+mwp*zu, (mw+mwp*zw)*u0, mq+mwp*u0,   -g*sin(tt0)*mwp, 0;
   0,         0,              1,            0,              0;
   0,        -u0,             0,            u0,             0;];

%matriz de entrada
B=[xde,         xdf;
   zde/u0,      zdf/u0;
   mde+mwp*zde, mdf+mwp*zdf;
   0,           0;
   0,           0];

BE=B(:,1); %selecionar entrada sempre no elevator


%--------- SAE: Realimentação do ângulo de picada (tt) para aumento do amortecimento da fugóide----------%

%damp(A)
%rlocus(A, BE, [0 0 0 -1 0], 0); %realimentação positiva do tt, com o anel fechado anterior
ktt=0.554; %critério: uma vez que o planador já é N1 nas condições dadas, colocar ksi_fug=0.6 sem alterar significativamente o PC
%damp(A+ktt*BE*[0 0 0 1 0])
Af=A+ktt*BE*[0 0 0 1 0];  %novo Anel Fechado, agora considerando também a realimentação de tt
sys=ss(Af, BE, [0 -1 0 1 0], 0); %sistema com realimentação SAE e saída gg

%------------Controlo de atitude/trajetória: servomecanismo com lqi---------------%

v=[1.53e-5 14.6 3.65 36.65 6.25e-9 3.65]; %diagonal da matriz Q
Q=diag(v);
R=4.19;
%Kc=lqi(sys,Q,R)  %Função em princípio só resolve no Matlab 2019, solução em baixo, se
%necessário podemos demonstrar que funciona
Kc=[-0.0196 1.6622 -0.8045 -3.0813 -0.0000 0.9333];


