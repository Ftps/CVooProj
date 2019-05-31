coord= [38.91889 -8.7725;
        38.75    -8.701944;
        38.83278 -8.436111;
        38.77139 -8.23;
        38.78167 -8.190833;
        38.79056 -8.186944;
        38.79639 -8.2;
        38.79889 -8.205];
    
[pontos(:,1), pontos(:,2)] = geodetic2enu(coord(:,1),coord(:,2), 0, coord(1,1), coord(1,2), 3048, wgs84Ellipsoid);
global pontos_T 
pontos_T = pontos.';