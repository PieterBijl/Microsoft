function H_int = H_int(fx, fy, xc, yc, zc)
    H_int = zeros(2,3);
    H_int(1,1) = fx/zc;
    H_int(2,2) = fy/zc;
    H_int(1,3) = -fx*xc/zc^2;
    H_int(2,3) = -fy*yc/zc^2;
end