
exec("gripen_lqr_sem_rumo.sci", -1);

function d = dist(x1, y1, x2, y2)
    d = sqrt((x1-x2)^2 + (y1-y2)^2);
endfunction

function p = delta_angle(x1, y1, x2, y2, psi)
    psi_f = acos((x2-x1)/dist(x1, y1, x2, y2));
    
    if y2 < y1 then
        psi_f = psi_f + %pi;
    end
    psi = pmodulo(psi, 2*%pi)
    p = psi_f - psi;
    
    if abs(p) > %pi then
        
    end
    
endfunction

function [ref, i] = patrulha(N, E, i, psi)
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
                    0, -4];
    d = 200;
    
    if dist(N, E, patrol(i)(1), patrol(i)(2)) < dist then
        if i == 10 then
            exit();
        else
            i = i + 1;
        end
    end
    
    
    
endfunction
