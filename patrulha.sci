
exec("gripen_lqr.sci", -1);

x0 = 0;
y0 = 0;
global i;
i = 1;
Vn = 0;
Ve = 0;

r_min = (u0^2)/(g*tan(30*deg));    
patrol = r_min*[0, 0;
                3, 0;
                8, 0;
                8, 2;
                3, 2;
                3, 4;
                8, 4;
                8, 6;
                3, 6;
                -4, 0];
d = 200;
ramp_slope = %pi*r_min/u0;

function dos = dist(x1, y1, x2, y2)
    dos = sqrt((x1-x2)^2 + (y1-y2)^2);
endfunction

function p = delta_angle(x1, y1, x2, y2, psi)
    psi_f = acos((x2-x1)/dist(x1, y1, x2, y2));
    disp(psi_f, 'Init');
    if y2 < y1 then
        psi_f = 2*%pi-psi_f;
    end
    disp(psi_f, 'First Corr');
    psi = modulo(psi, 2*%pi)
    p = psi_f - psi;
    disp(p, 'Pre-Fin');
    if abs(p) > %pi then
        p = -(2*%pi-p);
    end
    if abs(p) > 4*u0/r_min then
        p = sign(p)*4*u0/r_min;
    end
    disp(p, 'Second Corr');
endfunction

function [ref] = patrulha(N, E, psi)
    global i;
    disp([N, E, i], 'Coord:');
    if dist(N, E, patrol(i,1), patrol(i,2)) < d then
        if i == 10 then
            i = 1;
        else
            i = i + 1;
        end
    end
    
    ref = psi + delta_angle(N, E, patrol(i,1), patrol(i,2), psi);
    disp([ref, psi, t], 'Ref e Atual');
    /*
    if or([i == 4, i == 8]) then
        disp('oof');
    elseif i == 6 then
        disp('yee');
    else
        
    end
    */
endfunction

xcos('Patrulha.zcos');
