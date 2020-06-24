function [H_q]=H_q(fx, fy, xc, yc, zc, q)
    H_q=H_int(fx, fy, xc, yc, zc)*H_ext(q);
end