
exec("gripen_lqr.sci", -1);

x0 = 0;
y0 = -5000;

Vn = 0;
Ve = 0;

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

function [ref, i] = patrulha(N, E, j, psi)
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
    
    i = j;
    
    if dist(N, E, patrol(j)(1), patrol(j)(2)) < dist then
        if j == 10 then
            return;
        else
            i = j + 1;
            j = i;
        end
    end
    
    ref = delta_angle(N, E, patrol(j)(1), patrol(j)(2), psi);
    
endfunction

map_t = [0, 50, 98.5, 178.5, 227, 307, 355.5, 435.5, 500, 530, 600];
map_psi = [0, 0, %pi, %pi, 0, 0, %pi, %pi, %pi, 4*%pi/3, 4*%pi/3];

function i = teste(i)
    i = i+1;
endfunction
