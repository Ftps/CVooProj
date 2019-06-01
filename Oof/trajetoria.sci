function [psirefcorr]=trajetoria(E,N,t)
    function [psir]=nl(psii)
        psir=atan(52*sin(psii)/(52*cos(psii)-10))
    endfunction
    psiref=45;
    if N<1800 & E<1300                    then psiref=0;bbref=atan(2/3)
    elseif N>1800 & N<4800 & E<1300       then psiref=0; bbref=0;
    elseif N>4800 & E<2850                then psiref=180*deg; bbref=0;
    else                                       psiref=0; bbref=0;
    end
    psirefcorr=(fsolve(psiref),nl)
endfunction
