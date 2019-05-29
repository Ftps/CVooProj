clear;
clc;

time=100;

// s_chipmunk : flight condition : 2

deg=%pi/180; g=9.81
h=1500; M=0.16; aa0=3.69*deg; gg0=0; u0=101.5*(463/900); flaps=0; //aa0 convertido para rad; u0 convertido para m/s
//throttle : 
th0=41/100; de0=0.00; da0=0.00; dr0=0.00; //th0 convertido para %

Teng=0.10; demax=+28/-21; damax=17; drmax=23; flapmax=40;

//inertial data :
m=713; Ix=7227; Iy=4357; Iz=8700; Ixz=0;

//wing data : 
S=10.41; b=7.493; c=0.931; aamax=17.93;

//derivatives ( no units or SI units ):
xu=-0.0417; xw=0.1037; zu=-0.3742; zw=-3.0096; zwp=-0.0109; zq=-1.3098; mu=0.0000; mw=-0.1647; mq=-0.8679; mwp=0.0000;
ybb=-0.0592; lbb=-4.5691; nbb=1.3916; yp=0.0007; lp=-3.5254; np=-0.0177; yr=0.0057; lr=-0.0606; nr=-0.1928;
xde=0.000; zde=-1.309; mde=-1.884; xdf=-8.329; zdf=-8.444; mdf=0.000; xdt=2.681; zdt=0.000; mdt=0.000;
Lda=-20.931; Nda=0.120; Ydr=-0.018; Ldr=0.000; Ndr=-0.759;
   
    //Início do Programa Principal
    //Determinação do modelo
    
a=[ybb,                    yp+sin(aa0),                  yr-1,                 g/u0;
   lbb+((Ixz/Ix)*nbb)      lp+((Ixz/Ix)*np),             lr+((Ixz/Ix)*nr)         0;
   nbb+((Ixz/Ix)*lbb),     np+((Ixz/Ix)*lp),             nr+((Ixz/Ix)*lr),        0;
   0,                      1,                            0,                       0];
   
   
b=[0,       Ydr;
   Lda,     Ldr;
   Nda,     Ndr;
   0,         0];
   
c=eye(4,4);

    //Valores proprios, wn e amortecimento
[s1]=syslin('c',a,b,c) //espaço de estados
evals=spec(a)
[wn z]=damp(evals)
disp('polos; wn e z do sistema aberto')
disp(evals,[wn,z])

    //função transferência
[tf]=ss2tf(s1)
tfda=tf(:,1)
tfdr=tf(:,2)

    //Sistema de aumento de estabilidade; realimentacao positiva de r para o δR

c2=[0 0 -1 0];
[s2]=syslin('c',a,b(:,2),c2); 
[tf2]=ss2tf(s2);


    kk=-2.5
    
    k=[0 0 0 0;0 0 kk 0];

    af=a-b*k;
evals2=spec(af);
[wn2,z2]=damp(evals2);


[s3]=syslin('c',af,b(:,2),c2)
[tf3]=ss2tf(s3);
evals3=spec(af);
[wn3,z3]=damp(evals3);
disp('polos; wn e z do sistema realimentado')
disp(evals3,[wn3,z3])

//apercebemo-nos que a realimentação até diminui a oscilação das saidas, mas não é suficiente para colocar o RH no nivel 1.
//procedemos ao tratamento do seguimento de modelo por LQR

Q=[1/(1*deg)^2,     0,                 0,                            0;
   0,               1/(8*deg)^2,     0,                            0;
   0,               0,                 1/(8*deg)^2,                0;
   0,               0,                 0,                 1/(5*deg)^2];
   
R=[1/(4*deg)^2,    0;
   0,     1/(4*deg)^2];
   
S=zeros(2,4);

d=zeros(4,2);

SL=syslin('c',a,b,c,d)

[K,X]=lqr(SL,Q,R,S);

    AF_lqr=a+b*K;
SLlqr=syslin('c',AF_lqr,b,c,d)
evalslqr=spec(AF_lqr);
[wn,z]=damp(SLlqr)

disp('Resultados do LQR')
disp('novos polos')
disp(evalslqr)
disp('frequencias angulares e coef. de amort. com lqr')
disp([wn,z])

//controlo de atitude, matriz G e F

C=[1 0 0 0;0 0 0 1]
G=-C*inv(AF_lqr)*b;

   disp(G,"Ganho estático =");
        
   F=inv(G);
   
   disp(F,"Matriz F =");
