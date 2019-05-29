/*
Projeto 42 de CVoo:
    - Francisco dos Santos, 86631
    - Miguel Morgado, 86668
*/

function p = poles_i(ss)
    [wn, z] = damp(ss)
    p = zeros(size(wn, '*'), 1);
    for i = 1:size(wn, '*');
        p(i) = -wn(i)*(z(i) + %i*sqrt(1-z(i)^2));
    end
endfunction

fp = './Feed-Back Loop System.zcos'

g = 9.81;
// --jas39 : flight condition : 1
h =50; M =0.25; aa0 =3.69; gg0 =0; u0 =165.1; flaps =8; theta0 = aa0+gg0;
Teng =0.50; demax =[-22, 28]; damax =18; drmax =23; flapmax =40;
th0 =29; de0 =0.00; da0 =0.00; dr0 =0.00; g=9.81;
//inertial data :
m =10049; Ix =1434311; Iy =65079; Iz =1385502; Ixz =1763;
//wing data :
S =25.55; b =8.382; c =2.235; aamax =19.69;
//derivatives (no units or SI units ):
xu=-0.0434; xw=0.0741; zu=-0.2457; zw=-0.7104; zwp=-0.0046; zq=-1.0386; mu=0.0000; mw=-0.0557; mq=-0.7485; mwp=-0.0365;
ybb=-0.0438; lbb=-0.0379; nbb=0.1723; yp=0.0007; lp=-0.7252; np=0.0017; yr=0.0019; lr=0.0073; nr=-0.0222;
xde=0.000; zde=-0.002; mde=-0.003; xdsp=0.000; zdsp=0.000; mdsp=-0.077; xdt=7.650; zdt=0.000; mdt=0.000;
Lda=-0.384; Nda=-0.029; Ydr=-0.001; Ldr=-0.000; Ndr=-0.003;
tt0=gg0+aa0; //theta_zero

A = [ybb, yp+aa0, yr-1, (g/u0)*cos(tt0), 0;
    lbb+(Ixz/Ix)*nbb, lp+(Ixz/Ix)*np, lr+(Ixz/Ix)*nr, 0, 0;
    nbb+(Ixz/Iz)*lbb, np+(Ixz/Iz)*lp, nbb+(Ixz/Iz)*lbb, 0, 0;
    0, 1, tan(tt0), 0, 0;
    0, 0, 1/cos(tt0), 0, 0];
B=  [0, Ydr;
    Lda+(Ixz/Ix)*Nda, Ldr+(Ixz/Ix)*Ndr;
    Nda+(Ixz/Iz)*Lda, Ndr+(Ixz/Iz)*Ldr;
    0, 0;
    0, 0];

C = diag([0,1,0,0,0]);


ee=syslin('c',A,B,C) //ee=espaço de estados
[wn, z] = damp(ee);  //dá os Wn e os qsi dos 5 pólos

disp("Polos do sistema sem.controlador");
p = poles_i(ee);
disp(p);
//plzr(ee) // função que te desenha os pólos (e zeros) no plano complexo
[omegaN,z]=damp(ee)
T_eq=1./(omegaN.*z)

K=[0,0,0,0,0;
    0,0,0,0,0;
    0,-1,0,0,0;
    0,0,0,0,0;
    0,0,0,0,0]

sae=syslin('c',A-K*C,B,C);
[Wn, xi] = damp(sae);
disp(poles_i(sae))

h=ss2tf(ee)
