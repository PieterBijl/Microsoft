function H_ext=H_ext(q)
H_ext=-R_quat(q)*dR_dq(q);
end