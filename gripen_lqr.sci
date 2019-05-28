/*
Projeto de CVoo:
    - Francisco dos Santos, 86631
    - Miguel Morgado, 86668
*/

/*
depois descobri que havia a função poles(sys) que fazia o que nós queriamos automaticamente
mas ao menos assim deu para ver a sintaxe do scilab e o facto nojento de matrizes serem m() e nao m[]
poles(sys) e poles_i(sys) dão o mesmo resultado so it works
*/
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

fp = './Feed-Back Loop System.zcos'

max_x = [5*%pi/180, 3, 3, 20*%pi/180, 2*%pi];     // Valores máximos para os estados x e entradas u (Bryson)
max_u = [18*%pi/180, 23*%pi/180];

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


C = diag([1,1,1,1,1]);


ee=syslin('c',A,B,C) //ee=espaço de estados
[wn, z] = damp(ee);  //dá os Wn e os qsi dos 5 pólos

disp("Polos do sistema sem.controlador");
p = poles_i(ee);
disp(p);
// plzr(ee) --> função que te desenha os pólos (e zeros) no plano complexo

// Para trabalhar com o XCos (simulink do SciLab), ver links em baixo:
// https://steemit.com/utopian-io/@svozkan/simple-control-system-design-with-xcos-scilab-tutorial
// https://steemit.com/utopian-io/@svozkan/xcos-modelling-and-simulation-scilab-tutorial

// https://help.scilab.org/docs/5.5.2/en_US/lqr.html <- how to LQR in SciLab

// x = [bb, p, r, phi, psi]^T; u = [dA, dR]^T;
Q = get_Mat(max_x);               // Matriz de custo para o vetor de estados - ambos iniciados randomicamente
R = get_Mat(max_u);               // Matriz de custo para o vetor de entradas - for testing purposes   diag([1, 5, 0.3, 2, 3]);
                                // Posteriormente usar método de Bryson       diag([2, 1]); <- feito com o max_x e max_u

Big=sysdiag(Q,R);
[w,wp]=fullrf(Big);C1=wp(:,1:5);D12=wp(:,6:$);
                                    //   ^^  6:$ restante da matriz, vetor de entradas
                        // ^^  1:5 dimensão do vetor de estado, (3 variaveis de estado, 1:3)
                        // ver o link em cima para perceber melhor

P=syslin('c',A,B,C1,D12);
[K,X]=lqr(P);

K = -K                  // eles aqui definem o K para estar alimentado positivamente, assim está de acordo com a sebenta
G = -C*inv(A-B*K)*B;
F = pinv(G);            // something is wrong with this, ver sebenta a matriz F do LQR

norm(A'*X+X*A-X*B*inv(R)*B'*X+Q,1)

disp("Polos do sistema apos LQR");
pol = spec(A-B*K); // polos do sistema em LQR
disp(pol);

xcos(fp);

/* código para desenhar pólos do sistema apos o LQR no plano complexo:
Z=syslin('c',A-B*K,B,C);
plzr(Z)
[omegaN,z]=damp(pol)
*/
