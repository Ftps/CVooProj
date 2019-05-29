function [bbref,phiref]=trajetoria(E,N,bb,phi,lamda)
    //CONDIÇÕES INICIAIS
    rmin=600;
    Uw=0;
    lamda0=0*deg;
    N0=(0.5/2.8)*3*rmin;
    E0=0;
    while N<3*rmin
    end
    if N<3*rmin & E<3.5*rmin
        phiref=0;
        bbref=15*deg;
    end
bbref=0;
phiref=0;
endfunction
