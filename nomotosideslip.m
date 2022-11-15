function [X_output] = nomotosideslip(X_angle,del,T_tilde)

a_beta=-0.14;  %% 1.1m/s일때 identification으로 얻은 parameter value, nomoto model 계산용 matrix
b_beta=-0.2;
a_r=-0.34;
b_r=-1.06;
b_beta_bias=0.061;
b_r_bias=0.29;

A_mat=[a_beta -a_beta 1+b_beta;
    0 0 1;
    0 0 a_r];

B_mat=[0;
    0;
    b_r];

C_mat=[b_beta_bias;
    0;
    b_r_bias];

X_output=A_mat*X_angle'+B_mat*T_tilde*del+C_mat;

end