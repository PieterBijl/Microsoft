function x = EPnP2state_vector(x3d_h,x2d_h,A)
    [Rp,Tp,Xc,sol]=efficient_pnp(x3d_h,x2d_h,A);
    x = zeros(11,1);
    x(1:3) = Tp;
    x(7:10) = rotm2quat(Rp);
%     x(11:13) = [-0.0873;-0.1489;0.0262];
end