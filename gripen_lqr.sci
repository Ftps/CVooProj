/*
Projeto de CVoo:
    - Francisco dos Santos, 86631
    - Miguel Morgado, 86668
*/

/*
substituir poles_i por spec
*/

clear

function p = poles_i(ss)
    [wn, z] = damp(ss)
    p = zeros(size(wn, '*'), 1);
    for i = 1:size(wn, '*');
        p(i) = -wn(i)*(z(i) + %i*sqrt(1-z(i)^2));
    end
endfunction

function M = get_Mat(v)     // função a ser usada no metodo de Bryson para determinar as matrizes Q e R
    s = size(v, '*');
    M = zeros(s, s)
    for i = 1:s
        M(i,i) = 1/v(i)^2;
    end
endfunction

function F = stupid(A)
    F = diag([1,1]);
    disp(F);
    F = [1/A(1,1), 0; 0, 1]*F;
    disp(F);
    F = [1, 0; -c, 1]*F;
    disp(F);
    F = [1, 0; 0, 1/(A(2,2)-A(2,1)*A(1,2)/A(1,1))]*F;
    disp(F);
    F = [1,-A(1,1)/A(1,2); 0, 1]*F;
    disp(F);
endfunction

fp = 'Feed-Back Loop System.zcos'

g = 9.81; deg=%pi/180; kts=0.5144444444;
// --jas39 : flight condition : 1
h =50; M =0.25; aa0 =3.69*deg; gg0 =0; u0 =165.1*kts; flaps =8*deg; theta0 = aa0+gg0;
Teng =0.50; demax =[-22*deg, 28*deg]; damax =18*deg; drmax =23*deg; flapmax =40*deg;
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

Vn = 0;
Ve = 0;
x0 = 0;
y0 = 0;

A = [ybb, yp+aa0, yr-1, (g/u0)*cos(tt0), 0;
    lbb+(Ixz/Ix)*nbb, lp+(Ixz/Ix)*np, lr+(Ixz/Ix)*nr, 0, 0;
    nbb+(Ixz/Iz)*lbb, np+(Ixz/Iz)*lp, nr+(Ixz/Iz)*lr, 0, 0;
    0, 1, tan(tt0), 0, 0;
    0, 0, 1/cos(tt0), 0, 0];
B=  [0, Ydr;
    Lda+(Ixz/Ix)*Nda, Ldr+(Ixz/Ix)*Ndr;
    Nda+(Ixz/Iz)*Lda, Ndr+(Ixz/Iz)*Ldr;
    0, 0;
    0, 0];


C = diag([1,1,1,1,1]);


/*     ee=syslin('c',A,B,C) //ee=espaço de estados
[wn, z] = damp(ee);  //dá os Wn e os qsi dos 5 pólos

disp("Polos do sistema sem controlador");
p = poles_i(ee);
disp(p);      */
// plzr(ee) --> função que te desenha os pólos (e zeros) no plano complexo

// Para trabalhar com o XCos (simulink do SciLab), ver links em baixo:
// https://steemit.com/utopian-io/@svozkan/simple-control-system-design-with-xcos-scilab-tutorial
// https://steemit.com/utopian-io/@svozkan/xcos-modelling-and-simulation-scilab-tutorial

// https://help.scilab.org/docs/5.5.2/en_US/lqr.html <- how to LQR in SciLab

max_x = [5*deg, 0.4*deg, 0.75*deg, 0.2*deg, 0.05*deg];     // Valores máximos para os estados x e entradas u (Bryson)
max_u = [15*deg, 10*deg]; //temos depois de limitar isto à saida do sistema com um thresholdd

// x = [bb, p, r, phi, psi]^T; u = [dA, dR]^T;
Q = get_Mat(max_x)/100;               // Matriz de custo para o vetor de estados - ambos iniciados randomicamente
R = get_Mat(max_u)/100;               // Matriz de custo para o vetor de entradas - for testing purposes   diag([1, 5, 0.3, 2, 3]);
                                // Posteriormente usar método de Bryson       diag([2, 1]); <- feito com o max_x e max_u
                                
//Aqui começa o lqr(A,B,C1,D12)
Big=sysdiag(Q,R);
[w,wp]=fullrf(Big);C1=wp(:,1:5);D12=wp(:,6:$);
                                    //   ^^  6:$ restante da matriz, vetor de entradas
                        // ^^  1:5 dimensão do vetor de estado, (3 variaveis de estado, 1:3)

P=syslin('c',A,B,C1,D12);
[K,X]=lqr(P);
// Acaba aqui. Estas linhs de codigo fazem o LQR no Scilab
K = -K;
C_1 = [0, 0, 0, 0, 1;
       1, 0, 0, 0, 0];

          // eles aqui definem o K para estar alimentado positivamente, assim está de acordo com a sebenta
G = -C_1*inv(A-B*K)*B;
F = inv(G);
//F = (10^7)*[-1.047, 1.1809; -2.818, 3.1785]

norm(A'*X+X*A-X*B*inv(R)*B'*X+Q,1)

disp("Polos do sistema apos LQR, wn e xi");
pol = spec(A-B*K); // polos do sistema em LQR

//xcos(fp);

/* código para desenhar pólos do sistema apos o LQR no plano complexo e obter as caracteristicas para as qualidades de voo:
Z=syslin('c',A-B*K,B,C);
plzr(Z)
[omegaN,z]=damp(pol);
T_eq=1./(omegaN.*z);
disp(z(1),"Xi do RH=")
disp(omegaN(1),"Wn do RH=")
disp(z(1)*omegaN(1),"Xi*Wn do RH=")
disp(T_eq(3),"T_eq do R+S=")
*/

Z=syslin('c',A-B*K,B,C);
//plzr(Z)
[omegaN,z]=damp(pol)
T_eq=1./(omegaN.*z)
disp([pol, omegaN, z]);

