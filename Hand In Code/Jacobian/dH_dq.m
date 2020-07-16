function dH_dq=dH_dq(q,xyz)
dH_dq=dH_dq_cw(q,xyz)*[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 -1];
end