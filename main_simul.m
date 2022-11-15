close all
clear all
%% 변수 정의 및 초기값 정의
u=1.2; %% 속도 초기값
v=0;
r=0;

u_2=1.2; %% 속도 초기값
v_2=0;
r_2=0;

x = 100;
y = 50;

x_2 = 50;
y_2 = 10;

x_3 = 10;
y_3 = 50;

x_4 = 10;
y_4 = 50;

x_5 = 10;
y_5 = 50;

V_d_2 = 0;
V_d_3 = 0;

ship_2_vec = [0 0];
ship_3_vec = [0 0];
ship_4_vec = [0 0];
ship_5_vec = [0 0];
 
u_3=1.2; %% 속도 초기값
v_3=0;
r_3=0;

u_4=1.2; %% 속도 초기값
v_4=0;
r_4=0;

u_5=1.2; %% 속도 초기값
v_5=0;
r_5=0;

udot=0; %% 속도 미분 초기값
vdot=0;
rdot=0;
psidot=0;
chidot=0;

udot_2=0; %% 속도 미분 초기값
vdot_2=0;
rdot_2=0;
psidot_2=0;
chidot_2=0;

udot_3=0; %% 속도 미분 초기값
vdot_3=0;
rdot_3=0;
psidot_3=0;
chidot_3=0;

udot_4=0; %% 속도 미분 초기값
vdot_4=0;
rdot_4=0;
psidot_4=0;
chidot_4=0;

udot_5=0; %% 속도 미분 초기값
vdot_5=0;
rdot_5=0;
psidot_5=0;
chidot_5=0;

udot_vec_2 = [0 0];
udot_vec_3 = [0 0];
udot_vec_4 = [0 0];
udot_vec_5 = [0 0];

psi=90;
chi=90;

psi_2=90;
chi_2=90;

psi_temp_2 = 90;
psi_temp_3 = 90;
psi_temp_4 = 90;
psi_temp_5 = 90;

psi_3=0;
chi_3=0;

psi_4=90;
chi_4=90;

psi_5=90;
chi_5=90;

x_2_trans_b = 0; 
y_2_trans_b = 0;

x_2_trans_vec_b = 0; 
y_2_trans_vec_b = 0;

x_3_trans_b = 0; 
y_3_trans_b = 0;

x_4_trans_b = 10; 
y_4_trans_b = -50;

x_5_trans_b = 40; 
y_5_trans_b = -50;

X_local=[100 100 90];
X_local_2=[140 30 90];
X_local_3=[80 20 90];
X_local_4=[140 60 90];
X_local_5=[60 50 90];

x_2_trans = X_local_2(1);
y_2_trans = X_local_2(2);
x_3_trans = X_local_3(1);
y_3_trans = X_local_3(2);
x_4_trans = X_local_4(1);
y_4_trans = X_local_4(2);
x_5_trans = X_local_5(1);
y_5_trans = X_local_5(2);

x_2_trans_vec = 0;
y_2_trans_vec = 0;

% x_2_trans = 0;
% y_2_trans = 0;
% x_3_trans = 0;
% y_3_trans = 0;
% x_4_trans = 0;
% y_4_trans = 0;
% x_5_trans = 0;
% y_5_trans = 0;

travel_distance_2 = 0;
travel_distance_3 = 0;

cte_2 = 0;
cte_3 = 0;
cte_4 = 0;
cte_5 = 0;

vpt_vec_2 = [0 0];
vpt_vec_3 = [0 0];
vpt_vec_4 = [0 0];
vpt_vec_5 = [0 0];

x_2_b = 50;
y_2_b = 10;

x_3_b = 10;
y_3_b = 50;

x_4_b = 10;
y_4_b = -55;

x_5_b = 140;
y_5_b = 50;

err_V_2_b = 0;
err_V_3_b = 0;
err_V_4_b = 0;
err_V_5_b = 0;

del=0;  %% delta 초기값
del_2=0;  %% delta 초기값
del_3=0;  %% delta 초기값
del_4=0;
del_5=0;

t=0; %% 시간 loop
dt=0.1;
i=1;
n=1;

err_b=0; %% 에러 초기값
err_b_2=0; %% 에러 초기값
err_b_3=0; %% 에러 초기값
err_b_4=0;
err_b_5=0;

psi_d = 0;
err = 0;

psi_d_2 = 90;
psi_df_2 = 90;
err_2 = 0;
err_n_2 = 0;
err_n_2_b = 0;

psi_d_3 = 90;
err_3 = 0;
err_n_3 = 0;
err_n_3_b = 0;

psi_d_4 = 0;
err_4 = 0;
err_n_4 = 0;
err_n_4_b = 0;

psi_d_5 = 90;
err_5 = 0;
err_n_5 = 0;
err_n_5_b = 0;

kp = -20; %% PD 제어기 튜닝 (yaw)
kd = -25;

kp_n = 5000; % PD 제어기 튜닝 (rpm)
kd_n = 0;

% kp_n = 0.1; % PD 제어기 튜닝 (rpm)
% kd_n = 0;

d_l = 20; %% follower 위치 변수
d_l_2 = 40;
theta_l = 225;
d_r = 20;
d_r_2 = 40;
theta_r = -45;
n=1;
err_d_2 = 0;
err_d_3 = 0;
err_d_4 = 0;
err_d_5 = 0;

%% state 정의 및 계산식
X_bow_point=[7.5 15 0];

T_tilde=0; %% 추력 관련 parameter
c_tilde=-1.42*10^(-4);
d_tilde=8.96*10^(-7);

a_r=-0.4;
b_r=-1.06;
b_r_bias=0.29;

del_n_cmd = 400;
del_n_cmd_2 = 400;
del_n_cmd_3 = 400;
del_n_cmd_4 = 400;
del_n_cmd_5 = 400;

del_n=400; %% propeller speed
del_n_2=400;
del_n_3=400;
del_n_4=400;
del_n_5=400;

a_u=-0.12; %% speed dynamics parameter
b_u_bias=0.07;

X_output=[udot vdot rdot]; %% 가속도 성분

ship_1_vec = [0 0];
ship_2_vec = [0 0];

clear local_his;
close all


%% waypoint tracking
for t=0:dt:70

ship_2 = [x_2, y_2];
ship_3 = [x_3, y_3];
ship_4 = [x_4, y_4];
ship_5 = [x_5, y_5];

target_2 = [x_2_trans y_2_trans];
target_3 = [x_3_trans y_3_trans];
target_4 = [x_4_trans y_4_trans];
target_5 = [x_5_trans y_5_trans];

% [Z_2,psi_d_2,point_vec_2,extra_2] = make_rep_field_single(ship_2,target_2,ship_3,ship_3_vec,err_d_3,ship_4,ship_4_vec,err_d_4,ship_5,ship_5_vec,err_d_5);
% [Z_3,psi_d_3,point_vec_3,extra_3] = make_rep_field_single(ship_3,target_3,ship_2,ship_2_vec,err_d_2,ship_4,ship_4_vec,err_d_4,ship_5,ship_5_vec,err_d_5);
% [Z_4,psi_d_4,point_vec_4,extra_4] = make_rep_field_single(ship_4,target_4,ship_2,ship_2_vec,err_d_2,ship_3,ship_3_vec,err_d_3,ship_5,ship_5_vec,err_d_5);
% [Z_5,psi_d_5,point_vec_5,extra_5] = make_rep_field_single(ship_5,target_5,ship_2,ship_2_vec,err_d_2,ship_3,ship_3_vec,err_d_3,ship_4,ship_4_vec,err_d_4);

[Z_2,psi_d_2,point_vec_2,extra_2] = potential_field_basic(ship_2,target_2,ship_3,ship_3_vec,err_d_3,ship_4,ship_4_vec,err_d_4,ship_5,ship_5_vec,err_d_5);
[Z_3,psi_d_3,point_vec_3,extra_3] = potential_field_basic(ship_3,target_3,ship_2,ship_2_vec,err_d_2,ship_4,ship_4_vec,err_d_4,ship_5,ship_5_vec,err_d_5);
[Z_4,psi_d_4,point_vec_4,extra_4] = potential_field_basic(ship_4,target_4,ship_2,ship_2_vec,err_d_2,ship_3,ship_3_vec,err_d_3,ship_5,ship_5_vec,err_d_5);
[Z_5,psi_d_5,point_vec_5,extra_5] = potential_field_basic(ship_5,target_5,ship_2,ship_2_vec,err_d_2,ship_3,ship_3_vec,err_d_3,ship_4,ship_4_vec,err_d_4);

% psi_d_3 = atan2(y_3_trans-y_3,x_3_trans-x_3)*(180/3.1415);
% psi_d_4 = atan2(y_4_trans-y_4,x_4_trans-x_4)*(180/3.1415);
% psi_d_5 = atan2(y_5_trans-y_5,x_5_trans-x_5)*(180/3.1415);

if abs(err_d_2) < 15
    psi_d_2 = atan2(vpt_vec_2(2),vpt_vec_2(1))*(180/3.1415)-15*tanh(cte_2*0.25);
end

if abs(err_d_3) < 15
    psi_d_3 = atan2(vpt_vec_3(2),vpt_vec_3(1))*(180/3.1415)-15*tanh(cte_3*0.25);
end

if abs(err_d_4) < 15
    psi_d_4 = atan2(vpt_vec_4(2),vpt_vec_4(1))*(180/3.1415)-15*tanh(cte_4*0.25);
end

if abs(err_d_5) < 15
    psi_d_5 = atan2(vpt_vec_5(2),vpt_vec_5(1))*(180/3.1415)-15*tanh(cte_5*0.25);
end

% [fx_2, fy_2] = gradient(Z_2,1);
% %[fx_3, fy_3] = gradient(Z_3,1);


psi_d = 90;
% end
if psi_d_2 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기
       psi_d_2 = psi_d_2 - 360; 
end  
if psi_d_2 < -180
       psi_d_2 = psi_d_2 + 360; 
end

if psi_d_3 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기
       psi_d_3 = psi_d_3 - 360; 
end  
if psi_d_3 < -180
       psi_d_3 = psi_d_3 + 360; 
end

if psi_d_4 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기
       psi_d_4 = psi_d_4 - 360; 
end  
if psi_d_4 < -180
       psi_d_4 = psi_d_4 + 360; 
end

if psi_d_5 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기
       psi_d_5 = psi_d_5 - 360; 
end  
if psi_d_5 < -180
       psi_d_5 = psi_d_5 + 360; 
end

X_output=[chidot psidot rdot]; %% nomoto sideslip 모델 output
X_angle=[chi psi r];  %% 선체 각도들
X_body=[u v r];  %% NED의 Position 정보

X_output_2=[chidot_2 psidot_2 rdot_2]; %% follower 2 nomoto 모델 파라메터 정의
X_angle_2=[chi_2 psi_2 r_2];  
X_body_2=[u_2 v_2 r_2];  

X_output_3=[chidot_3 psidot_3 rdot_3]; %% follower 3 nomoto 모델 파라메터 정의
X_angle_3=[chi_3 psi_3 r_3];  
X_body_3=[u_3 v_3 r_3];  

X_output_4=[chidot_4 psidot_4 rdot_4]; %% follower 4 nomoto 모델 파라메터 정의
X_angle_4=[chi_4 psi_4 r_4];  
X_body_4=[u_4 v_4 r_4];  

X_output_5=[chidot_5 psidot_5 rdot_5]; %% follower 2 nomoto 모델 파라메터 정의
X_angle_5=[chi_5 psi_5 r_5];  
X_body_5=[u_5 v_5 r_5];  

del_cmd = err*kp + kd*(err-err_b)/dt;  %% 에러값 계산한 del 값
del_cmd_2 = err_2*kp + kd*(err_2-err_b_2)/dt;  %% follower 2 에러 계산
del_cmd_3 = err_3*kp + kd*(err_3-err_b_3)/dt;  %% follower 3 에러 계산
del_cmd_4 = err_4*kp + kd*(err_4-err_b_4)/dt;  %% follower 3 에러 계산
del_cmd_5 = err_5*kp + kd*(err_5-err_b_5)/dt;  %% follower 3 에러 계산

if del_n_cmd > 1300  %% 에러 값 제한
    del_n_cmd = 1300;
end

if del_n_cmd_2 > 1300  %% 에러 값 제한
    del_n_cmd_2 = 1300;
end
if del_n_cmd_2 < -1000
    del_n_cmd_2 = -1000;
end

if del_n_cmd_3 > 1300  %% 에러 값 제한
    del_n_cmd_3 = 1300;
end
if del_n_cmd_3 < -1000
    del_n_cmd_3 = -1000;
end

if del_n_cmd_4 > 1300  %% 에러 값 제한
    del_n_cmd_4 = 1300;
end
if del_n_cmd_4 < -1000
    del_n_cmd_4 = -1000;
end

if del_n_cmd_5 > 1300  %% 에러 값 제한
    del_n_cmd_5 = 1300;
end
if del_n_cmd_5 < -1000
    del_n_cmd_5 = -1000;
end

if del_cmd > 30 %%  del의 제한 값
    del_cmd = 30;
end

if del_cmd < -30
    del_cmd = -30;
end

if del_cmd_2 > 30 %%  del의 제한 값
    del_cmd_2 = 30;
end

if del_cmd_2 < -30
    del_cmd_2 = -30;
end

if del_cmd_3 > 30 %%  del의 제한 값
    del_cmd_3 = 30;
end

if del_cmd_3 < -30
    del_cmd_3 = -30;
end

if del_cmd_4 > 30 %%  del의 제한 값
    del_cmd_4 = 30;
end

if del_cmd_4 < -30
    del_cmd_4 = -30;
end

if del_cmd_5 > 30 %%  del의 제한 값
    del_cmd_5 = 30;
end

if del_cmd_5 < -30
    del_cmd_5 = -30;
end


del = del + 0.07*(del_cmd - del);  % del dynamics
del_2 = del_2 + 0.07*(del_cmd_2 - del_2);  % follower 2 del dynamics
del_3 = del_3 + 0.07*(del_cmd_3 - del_3);  % follower 3 del dynamics
del_4 = del_4 + 0.07*(del_cmd_4 - del_4);  % follower 4 del dynamics
del_5 = del_5 + 0.07*(del_cmd_5 - del_5);  % follower 5 del dynamics


if del > 30 %%  del의 제한 값
    del = 30;
end

if del < -30
    del = -30;
end

if del_2 > 30 %%  del의 제한 값
    del_2 = 30;
end

if del_2 < -30
    del_2 = -30;
end

if del_3 > 30 %%  del의 제한 값
    del_3 = 30;
end

if del_3 < -30
    del_3 = -30;
end

if del_4 > 30 %%  del의 제한 값
    del_4 = 30;
end

if del_4 < -30
    del_4 = -30;
end

if del_5 > 30 %%  del의 제한 값
    del_5 = 30;
end

if del_5 < -30
    del_5 = -30;
end

% %del = 3.66;
% del = -5;
% 
% if i > 500
%    del_n_cmd = del_n_cmd + 10;
% %     del = -5;
% end
% 
% if i > 1000
%     del = 12.32;
% end

del_n = del_n + 0.2*(del_n_cmd - del_n);
del_n_2 = del_n_2 + 0.2*(del_n_cmd_2 - del_n_2);
del_n_3 = del_n_3 + 0.2*(del_n_cmd_3 - del_n_3);
del_n_4 = del_n_4 + 0.2*(del_n_cmd_4 - del_n_4);
del_n_5 = del_n_5 + 0.2*(del_n_cmd_5 - del_n_5);

% % if abs(err_d_3) > 1
% del_n_3 = 300;
% % end
% 
% % if abs(err_d_2) > 1
% del_n_2 = 300;
% % end


T_tilde=c_tilde*u*del_n+d_tilde*abs(del_n)*del_n;  %% nomoto에 쓰기위한 추력계산
T_tilde_2=c_tilde*u_2*del_n_2+d_tilde*abs(del_n_2)*del_n_2;
T_tilde_3=c_tilde*u_3*del_n_3+d_tilde*abs(del_n_3)*del_n_3;
T_tilde_4=c_tilde*u_4*del_n_4+d_tilde*abs(del_n_4)*del_n_4;
T_tilde_5=c_tilde*u_5*del_n_5+d_tilde*abs(del_n_5)*del_n_5;

udot=a_u*u+T_tilde+b_u_bias; %% speed dynamics
u=u+udot*dt;
% u =0;

udot_2=a_u*u_2+T_tilde_2+b_u_bias; %% speed dynamics
u_2 = u_2+udot_2*dt;

udot_3=a_u*u_3+T_tilde_3+b_u_bias; %% speed dynamics
u_3 = u_3+udot_3*dt;

udot_4=a_u*u_4+T_tilde_4+b_u_bias; %% speed dynamics
u_4 = u_4+udot_4*dt;

udot_5=a_u*u_5+T_tilde_5+b_u_bias; %% speed dynamics
u_5 = u_5+udot_5*dt;

udot_vec_2 = [udot_2*cosd(psi_temp_2) udot_2*sind(psi_temp_2)];
udot_vec_3 = [udot_3*cosd(psi_temp_3) udot_3*sind(psi_temp_3)];
udot_vec_4 = [udot_4*cosd(psi_temp_4) udot_4*sind(psi_temp_4)];
udot_vec_5 = [udot_5*cosd(psi_temp_5) udot_5*sind(psi_temp_5)];

[X_output] = nomotosideslip(X_angle,del,T_tilde); %% nomoto sideslip model로 angle계산
[X_output_2] = nomotosideslip(X_angle_2,del_2,T_tilde_2); %% follower 2 nomoto sideslip model로 angle계산
[X_output_3] = nomotosideslip(X_angle_3,del_3,T_tilde_3);
[X_output_4] = nomotosideslip(X_angle_4,del_4,T_tilde_4);
[X_output_5] = nomotosideslip(X_angle_5,del_5,T_tilde_5);

rdot=[0 0 1]*X_output; %% nomoto output 값에서 rdot 도출
psidot=[0 1 0]*X_output; %% nomoto output 값에서 psidot 도출
chidot=[1 0 0]*X_output; %% nomoto output 값에서 chidot 도출
r=r+rdot*dt; %% rdot 적분해서 r값 도출
psi=psi+r*dt; %% r 적분해서 psi 도출
chi = chi + chidot*dt;  %% chidot 적분해서 chi값 도출

rdot_2=[0 0 1]*X_output_2; %% nomoto output 값에서 rdot 도출 %% follower 2
psidot_2=[0 1 0]*X_output_2; %% nomoto output 값에서 psidot 도출
chidot_2=[1 0 0]*X_output_2; %% nomoto output 값에서 chidot 도출
r_2=r_2+rdot_2*dt; %% rdot 적분해서 r값 도출
psi_2=psi_2+r_2*dt; %% r 적분해서 psi 도출
chi_2 = chi_2 + chidot_2*dt;  %% chidot 적분해서 chi값 도출

rdot_3=[0 0 1]*X_output_3; %% nomoto output 값에서 rdot 도출 %% follower 2
psidot_3=[0 1 0]*X_output_3; %% nomoto output 값에서 psidot 도출
chidot_3=[1 0 0]*X_output_3; %% nomoto output 값에서 chidot 도출
r_3=r_3+rdot_3*dt; %% rdot 적분해서 r값 도출
psi_3=psi_3+r_3*dt; %% r 적분해서 psi 도출
chi_3 = chi_3 + chidot_3*dt;  %% chidot 적분해서 chi값 도출

rdot_4=[0 0 1]*X_output_4; %% nomoto output 값에서 rdot 도출 %% follower 2
psidot_4=[0 1 0]*X_output_4; %% nomoto output 값에서 psidot 도출
chidot_4=[1 0 0]*X_output_4; %% nomoto output 값에서 chidot 도출
r_4=r_4+rdot_4*dt; %% rdot 적분해서 r값 도출
psi_4=psi_4+r_4*dt; %% r 적분해서 psi 도출
chi_4 = chi_4 + chidot_4*dt;  %% chidot 적분해서 chi값 도출

rdot_5=[0 0 1]*X_output_5; %% nomoto output 값에서 rdot 도출 %% follower 2
psidot_5=[0 1 0]*X_output_5; %% nomoto output 값에서 psidot 도출
chidot_5=[1 0 0]*X_output_5; %% nomoto output 값에서 chidot 도출
r_5=r_5+rdot_5*dt; %% rdot 적분해서 r값 도출
psi_5=psi_5+r_5*dt; %% r 적분해서 psi 도출
chi_5 = chi_5 + chidot_5*dt;  %% chidot 적분해서 chi값 도출

X_body=[u v r];  %% NED의 Position 정보
X_body_2=[u_2 v_2 r_2];  %% NED의 Position 정보
X_body_3=[u_3 v_3 r_3];  %% NED의 Position 정보
X_body_4=[u_4 v_4 r_4];  %% NED의 Position 정보
X_body_5=[u_5 v_5 r_5];  %% NED의 Position 정보

err_b=err;  %% 직전 에러값 저장
err_b_2=err_2;  %% 직전 에러값 저장
err_b_3=err_3;  %% 직전 에러값 저장
err_b_4=err_4;  %% 직전 에러값 저장
err_b_5=err_5;  %% 직전 에러값 저장

psi_temp = rem(psi,360);
if psi_temp > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기
       psi_temp = psi_temp - 360; 
end  
if psi_temp < -180
       psi_temp = psi_temp + 360; 
end

psi_temp_2 = rem(psi_2,360);
if psi_temp_2 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기 %% follower 2
       psi_temp_2 = psi_temp_2 - 360; 
end  
if psi_temp_2 < -180
       psi_temp_2 = psi_temp_2 + 360; 
end

psi_temp_3 = rem(psi_3,360);
if psi_temp_3 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기 %% follower 2
       psi_temp_3 = psi_temp_3 - 360; 
end  
if psi_temp_3 < -180
       psi_temp_3 = psi_temp_3 + 360; 
end

psi_temp_4 = rem(psi_4,360);
if psi_temp_4 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기 %% follower 2
       psi_temp_4 = psi_temp_4 - 360; 
end  
if psi_temp_4 < -180
       psi_temp_4 = psi_temp_4 + 360; 
end

psi_temp_5 = rem(psi_5,360);
if psi_temp_5 > 180  %% 프사이 값 제한해서 빙빙 돌며 추적하지 않게 하기 %% follower 2
       psi_temp_5 = psi_temp_5 - 360; 
end  
if psi_temp_5 < -180
       psi_temp_5 = psi_temp_5 + 360; 
end

err = psi_d-psi_temp;  %% 에러 정의
err_2 = psi_d_2-psi_temp_2;  %% follower 2 에러 정의
err_3 = psi_d_3-psi_temp_3;  %% follower 3 에러 정의
err_4 = psi_d_4-psi_temp_4;  %% follower 3 에러 정의
err_5 = psi_d_5-psi_temp_5;  %% follower 3 에러 정의

if err > 180  %% 에러 값 제한
    err = err - 360;
end
if err < -180
    err = err + 360;
end

if err_2 > 180  %% 에러 값 제한
    err_2 = err_2 - 360;
end
if err_2 < -180
    err_2 = err_2 + 360;
end

if err_3 > 180  %% 에러 값 제한
    err_3 = err_3 - 360;
end
if err_3 < -180
    err_3 = err_3 + 360;
end

if err_4 > 180  %% 에러 값 제한
    err_4 = err_4 - 360;
end
if err_4 < -180
    err_4 = err_4 + 360;
end

if err_5 > 180  %% 에러 값 제한
    err_5 = err_5 - 360;
end
if err_5 < -180
    err_5 = err_5 + 360;
end

X_dot = [u*cosd(psi) u*sind(chi) r];
X_dot_2 = [u_2*cosd(chi_2) u_2*sind(chi_2) r_2];
X_dot_3 = [u_3*cosd(chi_3) u_3*sind(chi_3) r_3];
X_dot_4 = [u_4*cosd(chi_4) u_4*sind(chi_4) r_4];
X_dot_5 = [u_5*cosd(chi_5) u_5*sind(chi_5) r_5];

X_local=X_local+X_dot*dt;
X_local_2=X_local_2+X_dot_2*dt;
X_local_3=X_local_3+X_dot_3*dt;
X_local_4=X_local_4+X_dot_4*dt;
X_local_5=X_local_5+X_dot_5*dt;

x = X_local(1);
y = X_local(2);

x_2 = X_local_2(1);
y_2 = X_local_2(2);

x_3 = X_local_3(1);
y_3 = X_local_3(2);

x_4 = X_local_4(1);
y_4 = X_local_4(2);

x_5 = X_local_5(1);
y_5 = X_local_5(2);

local_his(i,:)=X_local;
psi_d_his(i,:)=psi_d;
psi_his(i,:)=psi;

local_his_2(i,:)=X_local_2;
local_his_3(i,:)=X_local_3;
local_his_4(i,:)=X_local_4;
local_his_5(i,:)=X_local_5;
psi_d_his_2(i,:)=psi_d_2;
psi_his_2(i,:)=psi_temp_2;

%% follower 목표점 계산

x_2_des = x + d_l*cosd(theta_l) - x;
y_2_des = y + d_l*sind(theta_l) - y;

% if t > 150
%     x_2_des = 0;
%     y_2_des = -20;
% end
% 
% if t > 250
%     x_2_des = -20;
%     y_2_des = 0;
% end

x_3_des = x + d_r*cosd(theta_r) - x;
y_3_des = y + d_r*sind(theta_r) - y;

% if t > 150
%     x_3_des = 0;
%     y_3_des = -40;
% end
% 
% if t > 250
%     x_3_des = 20;
%     y_3_des = 0;
% end

x_2_trans = x_2_des*cosd(psi-90) - y_2_des*sind(psi-90) + x;
y_2_trans = x_2_des*sind(psi-90) + y_2_des*cosd(psi-90) + y;

% x_2_trans_vec = x_2_trans + vpt_vec_2(1)*20;
% y_2_trans_vec = y_2_trans + vpt_vec_2(2)*20;
% 
% 
% x_2_trans_vec = x_2_trans;
% y_2_trans_vec = y_2_trans;


x_3_trans = x_3_des*cosd(psi-90) - y_3_des*sind(psi-90) + x;
y_3_trans = x_3_des*sind(psi-90) + y_3_des*cosd(psi-90) + y;


x_4_des = x + d_l_2*cosd(theta_l) - x;
y_4_des = y + d_l_2*sind(theta_l) - y;

% if t > 150
%     x_4_des = 0;
%     y_4_des = -60;
% end
% 
% if t > 250
%     x_4_des = -40;
%     y_4_des = 0;
% end

x_5_des = x + d_r_2*cosd(theta_r) - x;
y_5_des = y + d_r_2*sind(theta_r) - y;
% 
% if t > 150
%     x_5_des = 0;
%     y_5_des = -80;
% end
% 
% if t > 250
%     x_5_des = 40;
%     y_5_des = 0;
% end

x_4_trans = x_4_des*cosd(psi-90) - y_4_des*sind(psi-90) + x;
y_4_trans = x_4_des*sind(psi-90) + y_4_des*cosd(psi-90) + y;

x_5_trans = x_5_des*cosd(psi-90) - y_5_des*sind(psi-90) + x;
y_5_trans = x_5_des*sind(psi-90) + y_5_des*cosd(psi-90) + y;

ship_2_vec = [x_2 - x_2_b, y_2 - y_2_b];
ship_3_vec = [x_3 - x_3_b, y_3 - y_3_b];
ship_4_vec = [x_4 - x_4_b, y_4 - y_4_b];
ship_5_vec = [x_5 - x_5_b, y_5 - y_5_b];

vpt_vec_2 = [x_2_trans - x_2_trans_b, y_2_trans - y_2_trans_b];
u_vec_2 = [x_2 - x_2_trans, y_2 - y_2_trans];

vpt_vec_3 = [x_3_trans - x_3_trans_b, y_3_trans - y_3_trans_b];
u_vec_3 = [x_3 - x_3_trans, y_3 - y_3_trans];

vpt_vec_4 = [x_4_trans - x_4_trans_b, y_4_trans - y_4_trans_b];
u_vec_4 = [x_4 - x_4_trans, y_4 - y_4_trans];

vpt_vec_5 = [x_5_trans - x_5_trans_b, y_5_trans - y_5_trans_b];
u_vec_5 = [x_5 - x_5_trans, y_5 - y_5_trans];

dot_vec_2 = dot(vpt_vec_2,u_vec_2);
dot_vec_3 = dot(vpt_vec_3,u_vec_3);
dot_vec_4 = dot(vpt_vec_4,u_vec_4);
dot_vec_5 = dot(vpt_vec_5,u_vec_5);

err_d_2 = sqrt((x_2_trans-x_2)^2+(y_2_trans-y_2)^2)*sign(dot_vec_2);
err_d_3 = sqrt((x_3_trans-x_3)^2+(y_3_trans-y_3)^2)*sign(dot_vec_3);
err_d_4 = sqrt((x_4_trans-x_4)^2+(y_4_trans-y_4)^2)*sign(dot_vec_4);
err_d_5 = sqrt((x_5_trans-x_5)^2+(y_5_trans-y_5)^2)*sign(dot_vec_5);


cross_vec_2 = cross([vpt_vec_2 0]',[u_vec_2 0]');
cross_vec_3 = cross([vpt_vec_3 0]',[u_vec_3 0]');
cross_vec_4 = cross([vpt_vec_4 0]',[u_vec_4 0]');
cross_vec_5 = cross([vpt_vec_5 0]',[u_vec_5 0]');



cte_2 =real(sqrt(sqrt((x_2 - x_2_trans)^2+(y_2 - y_2_trans)^2)^2 - dot_vec_2^2))* sign(cross_vec_2(3));
cte_3 =real(sqrt(sqrt((x_3 - x_3_trans)^2+(y_3 - y_3_trans)^2)^2 - dot_vec_3^2))* sign(cross_vec_3(3));
cte_4 =real(sqrt(sqrt((x_4 - x_4_trans)^2+(y_4 - y_4_trans)^2)^2 - dot_vec_4^2))* sign(cross_vec_4(3));
cte_5 =real(sqrt(sqrt((x_5 - x_5_trans)^2+(y_5 - y_5_trans)^2)^2 - dot_vec_5^2))* sign(cross_vec_5(3));

V_d_2 = sqrt((x_2_trans - x_2_trans_b)^2+(y_2_trans - y_2_trans_b)^2)/dt - 2*tanh(dot(vpt_vec_2,u_vec_2));
V_d_3 = sqrt((x_3_trans - x_3_trans_b)^2+(y_3_trans - y_3_trans_b)^2)/dt - 2*tanh(dot(vpt_vec_3,u_vec_3));
V_d_4 = sqrt((x_4_trans - x_4_trans_b)^2+(y_4_trans - y_4_trans_b)^2)/dt - 2*tanh(dot(vpt_vec_4,u_vec_4));
V_d_5 = sqrt((x_5_trans - x_5_trans_b)^2+(y_5_trans - y_5_trans_b)^2)/dt - 2*tanh(dot(vpt_vec_5,u_vec_5));

err_V_2 = V_d_2 - u_2;
err_V_3 = V_d_3 - u_3;
err_V_4 = V_d_4 - u_4;
err_V_5 = V_d_5 - u_5;

del_n_cmd_2 = 400 + kp_n*err_V_2 + kd_n*(err_V_2-err_V_2_b)/dt;
del_n_cmd_3 = 400 + kp_n*err_V_3 + kd_n*(err_V_3-err_V_3_b)/dt;
del_n_cmd_4 = 400 + kp_n*err_V_4 + kd_n*(err_V_4-err_V_4_b)/dt;
del_n_cmd_5 = 400 + kp_n*err_V_5 + kd_n*(err_V_5-err_V_5_b)/dt;

x_2_trans_b = x_2_trans;
y_2_trans_b = y_2_trans;

x_3_trans_b = x_3_trans;
y_3_trans_b = y_3_trans;

x_4_trans_b = x_4_trans;
y_4_trans_b = y_4_trans;

x_5_trans_b = x_5_trans;
y_5_trans_b = y_5_trans;


% travel_distance_2 = travel_distance_2 + abs(sqrt((x_2-x_2_b)^2+(y_2-y_2_b)^2));
% travel_distance_3 = travel_distance_3 + abs(sqrt((x_3-x_3_b)^2+(y_3-y_3_b)^2));
% 
% if abs(err_d_2) < 3
%     travel_distance_2 = 0;
% end
% 
% if abs(err_d_3_tran) < 3
%     travel_distance_3 = 0;
% end

x_2_b = x_2;
y_2_b = y_2;

x_3_b = x_3;
y_3_b = y_3;

x_4_b = x_4;
y_4_b = y_4;

x_5_b = x_5;
y_5_b = y_5;

err_V_2_b = err_V_2;
err_V_3_b = err_V_3;
err_V_4_b = err_V_4;
err_V_5_b = err_V_5;

% d_bet_ship = sqrt((x_3-x_2)^2+(y_3-y_2)^2);



del_his(i,:)=del;
del_n_his(i,:)=del_n;
psi_his(i,:)=psi;

del_his_2(i,:)=del_2;
del_n_his_2(i,:)=del_n_2;
psi_his_2(i,:)=psi_temp_2;%*3.1415/180;

del_his_3(i,:)=del_3;
del_n_his_3(i,:)=del_n_3;
psi_his_3(i,:)=psi_temp_3;%*3.1415/180;

psi_d_his_3(i,:)=psi_d_3;


trans_2_his(i,:)=[x_2_trans,y_2_trans];
trans_3_his(i,:)=[x_3_trans,y_3_trans];
trans_4_his(i,:)=[x_4_trans,y_4_trans];
trans_5_his(i,:)=[x_5_trans,y_5_trans];
trans_vec_2_his(i,:)=[x_2_trans_vec,y_2_trans_vec];


n_his(i,:)=n;
err_his(i,:)=err;
err_his_2(i,:)=err_2*3.1415/180;
err_his_3(i,:)=err_3*3.1415/180;

err_his_de_2(i,:)=err_2;
err_his_de_3(i,:)=err_3;

psi_d_his_2(i,:)=psi_d_2;
% psi_his_2(i,:)=psi_2;

err_V_2_his(i,:)=err_V_2;
% err_d_2_his(i,:)=err_d_2_tran;
% err_d_3_his(i,:)=err_d_3_tran;
u_2_his(i,:) = u_2;
del_n_cmd_2_his(i,:) = del_n_cmd_2;
del_n_cmd_3_his(i,:) = del_n_cmd_3;

cte_his(i,:)=40*tanh(cte_2*0.05);

del_cmd_his(i,:)=del_cmd;
u_his(i,:)=u;
follow_2_his(i,:)=[x_2_trans , y_2_trans];

u_scale = 50;

draw_2_3_x = u_scale*udot_vec_3(1);
draw_2_3_y = u_scale*udot_vec_3(2);

draw_2_4_x = u_scale*udot_vec_4(1);
draw_2_4_y = u_scale*udot_vec_4(2);

draw_2_5_x = u_scale*udot_vec_5(1);
draw_2_5_y = u_scale*udot_vec_5(2);

draw_4_2_x = u_scale*udot_vec_2(1);
draw_4_2_y = u_scale*udot_vec_2(2);

draw_5_2_x = u_scale*udot_vec_2(1);
draw_5_2_y = u_scale*udot_vec_2(2);

% err_d_2_his(i,:) = err_d_2;

trans_2_hhis(i,:) = [x_2_trans,y_2_trans];
trans_3_hhis(i,:) = [x_3_trans,y_3_trans];
trans_4_hhis(i,:) = [x_4_trans,y_4_trans];
trans_5_hhis(i,:) = [x_5_trans,y_5_trans];

trans_ve_hhis(i,:) = [x_2_trans_vec,y_2_trans_vec];

% travel_his_2(i,:) = travel_distance_2;
% travel_his_3(i,:) = travel_distance_3;
% 
% dist_bet_ship(i,:)=d_bet_ship; 




i=i+1;

end

%% figure
f1=figure(1);
f1.Position = [100 100 540 400];
% ss = surf(Z_4);
% ss.EdgeColor = 'none';
[fxx,fyy]= gradient(Z_5,1);
quiver(-fxx,-fyy);
hold on;
plot(local_his(:,1),local_his(:,2),'b')
hold on;
aa=plot(local_his_2(:,1),local_his_2(:,2),'r:','LineWidth',2);
hold on;
% quiver(-fx_2,-fy_2)
% aa.Color(4)=0.3;
aaa=plot(local_his_3(:,1),local_his_3(:,2),'k:','LineWidth',2);
plot(local_his_4(:,1),local_his_4(:,2),'c:','LineWidth',2);
plot(local_his_5(:,1),local_his_5(:,2),'g:','LineWidth',2);
dtt=1;
% for tt=1:dtt:1
% % del_in = del_his(tt*100-1,1);
% % n_in = del_n_his(tt*100-1,1);
% % pos_in = [local_his(tt*100-1,1) local_his(tt*100-1,2) psi_his(tt*100-1,1)];
% % 
% % [boat_pt] = boat_gui(del_in,n_in, pos_in);
% %     
% %     outline = boat_pt{1};
% %     inline = boat_pt{2};
% %     backboard = boat_pt{4};
% %     frontboard = boat_pt{3};
% %     motor_TRRot = boat_pt{6};
% %     motor_rot = boat_pt{5};
% %     {outline, inline, frontboard, backboard, motor_rot, motor_TRRot};
% %     a1 = fill(outline(:,1),outline(:,2),[0.1,0.1,0.1]);
% %     set(a1,'facecolor','b','facealpha',0.1,'edgecolor','none')
% %     hold on
% %     a2 = fill(inline(:,1),inline(:,2),[0.2,0.2,0.2]);
% %     set(a2,'facecolor','b','facealpha',0.05,'edgecolor','none')
% %     a3 = fill(backboard(:,1),backboard(:,2),[0.3,0.3,0.3]);
% %     set(a3,'facecolor','b','facealpha',0.05,'edgecolor','none')
% %     a4 = fill(frontboard(:,1),frontboard(:,2),[0.3,0.3,0.3]);
% %     set(a4,'facecolor','b','facealpha',0.05,'edgecolor','none')
% %     a5 = plot(motor_TRRot(:,1),motor_TRRot(:,2),'b','linewidth',4);
% %     a5.Color(4)=0.1;
% %     a6 = fill(motor_rot(:,1),motor_rot(:,2),[0.8,0.8,0.8]);
% %     set(a6,'facecolor','b','facealpha',0.05,'edgecolor','none')
%     
% del_in_2 = del_his_2(tt*100-1,1);
% n_in_2 = del_n_his_2(tt*100-1,1);
% pos_in_2 = [local_his_2(tt*100-1,1) local_his_2(tt*100-1,2) psi_his_2(tt*100-1,1)];
% 
% [boat_pt] = boat_gui(del_in_2,n_in_2, pos_in_2);
%     
%     outline = boat_pt{1};
%     inline = boat_pt{2};
%     backboard = boat_pt{4};
%     frontboard = boat_pt{3};
%     motor_TRRot = boat_pt{6};
%     motor_rot = boat_pt{5};
%     {outline, inline, frontboard, backboard, motor_rot, motor_TRRot};
%     a1 = fill(outline(:,1),outline(:,2),[0.1,0.1,0.1]);
%     set(a1,'facecolor','k','facealpha',1,'edgecolor','none')
%     hold on
%     a2 = fill(inline(:,1),inline(:,2),[0.2,0.2,0.2]);
%     set(a2,'facecolor','k','facealpha',1,'edgecolor','none')
%     a3 = fill(backboard(:,1),backboard(:,2),[0.3,0.3,0.3]);
%     set(a3,'facecolor','k','facealpha',1,'edgecolor','none')
%     a4 = fill(frontboard(:,1),frontboard(:,2),[0.3,0.3,0.3]);
%     set(a4,'facecolor','k','facealpha',1,'edgecolor','none')
%     a5 = plot(motor_TRRot(:,1),motor_TRRot(:,2),'k','linewidth',4);
%     a5.Color(4)=1;
%     a6 = fill(motor_rot(:,1),motor_rot(:,2),[0.8,0.8,0.8]);
%     set(a6,'facecolor','k','facealpha',1,'edgecolor','none')
%     
% del_in_3 = del_his_3(tt*100-1,1);
% n_in_3 = del_n_his_3(tt*100-1,1);
% pos_in_3 = [local_his_3(tt*100-1,1) local_his_3(tt*100-1,2) psi_his_3(tt*100-1,1)];
% 
% [boat_pt] = boat_gui(del_in_3,n_in_3, pos_in_3);
%     
%     outline = boat_pt{1};
%     inline = boat_pt{2};
%     backboard = boat_pt{4};
%     frontboard = boat_pt{3};
%     motor_TRRot = boat_pt{6};
%     motor_rot = boat_pt{5};
%     {outline, inline, frontboard, backboard, motor_rot, motor_TRRot};
%     a1 = fill(outline(:,1),outline(:,2),[0.1,0.1,0.1]);
%     set(a1,'facecolor','r','facealpha',1,'edgecolor','none')
%     hold on
%     a2 = fill(inline(:,1),inline(:,2),[0.2,0.2,0.2]);
%     set(a2,'facecolor','r','facealpha',1,'edgecolor','none')
%     a3 = fill(backboard(:,1),backboard(:,2),[0.3,0.3,0.3]);
%     set(a3,'facecolor','r','facealpha',1,'edgecolor','none')
%     a4 = fill(frontboard(:,1),frontboard(:,2),[0.3,0.3,0.3]);
%     set(a4,'facecolor','r','facealpha',1,'edgecolor','none')
%     a5 = plot(motor_TRRot(:,1),motor_TRRot(:,2),'r','linewidth',4);
%     a5.Color(4)=1;
%     a6 = fill(motor_rot(:,1),motor_rot(:,2),[0.8,0.8,0.8]);
%     set(a6,'facecolor','r','facealpha',1,'edgecolor','none')
% 
% 
% % plot(trans_2_his(tt*100-1,1),trans_2_his(tt*100-1,2),'g*')
% % plot(trans_3_his(tt*100-1,1),trans_3_his(tt*100-1,2),'g*')
% % plot(trans_4_his(tt*100-1,1),trans_4_his(tt*100-1,2),'g*')
% % plot(trans_5_his(tt*100-1,1),trans_5_his(tt*100-1,2),'g*')
% % plot(trans_vec_2_his(tt*100+10,1),trans_vec_2_his(tt*100+10,2),'c*')
% 
% 
% end

% aaa.Color(4)=0.3;

% plot(tilde_his(:,1))
hold on
% plot(WP(:,1),WP(:,2),'r*')
grid minor
plot(x_2_trans,y_2_trans,'g*')
plot(x_3_trans,y_3_trans,'g*')
plot(x_4_trans,y_4_trans,'g*')
plot(x_5_trans,y_5_trans,'g*')
% plot(x_2_trans_vec,y_2_trans_vec,'c*')
axis equal
%axis([220 265 7 42])
tit=title('Trajectory');
set(tit,'FontName', 'Times new roman', 'fontsize',20)
xla = xlabel('x (m)');
yla = ylabel('y (m)');
set(xla,'FontName', 'Times new roman','fontsize',20)
set(yla,'FontName', 'Times new roman','fontsize',20)


% ttimeett = 0.1;
% timee = 0:ttimeett:0.1*length(err_d_3_his)-0.1;
% timee_tr_2 = 0:ttimeett:0.1*length(travel_his_2)-0.1;
% timee_tr_3 = 0:ttimeett:0.1*length(travel_his_3)-0.1;
% 
% f2=figure(2)
% subplot(411)
% plot(timee,err_d_3_his(:,1),'r','LineWidth',2)
% hold on
% plot(timee,err_d_2_his(:,1),'b:','LineWidth',2)
% grid minor
% % axis([0 50 -10 80])
% 
% xla = xlabel('Time [sec]');
% yla = ylabel('D_e  [m]');
% tla=title('Euclidean Distance between ship & target point');
% set(tla,'FontName', 'Times new roman','fontsize',14)
% set(xla,'FontName', 'Times new roman','fontsize',12)
% set(yla,'FontName', 'Times new roman','fontsize',13)
% legend('D_e(PFRV)','D_e(PF)','fontsize',10)
% 
% subplot(412)
% plot(timee_tr_3,travel_his_3(:,1),'r','LineWidth',2)
% hold on
% plot(timee_tr_2,travel_his_2(:,1),'b:','LineWidth',2)
% grid minor
% % axis([0 50 0 80])
% 
% xla = xlabel('Time [sec]');
% yla = ylabel('D_t  [m]');
% tla = title('Travel Distance');
% set(tla,'FontName', 'Times new roman','fontsize',14)
% set(xla,'FontName', 'Times new roman','fontsize',12)
% set(yla,'FontName', 'Times new roman','fontsize',13)
% legend('D_t(PFRV)','D_t(PF)','fontsize',10)
% 
% subplot(413)
% plot(timee,psi_his_3(:,1),'r','LineWidth',2)
% hold on
% plot(timee,psi_his_2(:,1),'b:','LineWidth',2)
% grid minor
% % axis([0 50 -4 4])
% 
% xla = xlabel('Time [sec]');
% yla = ylabel('\psi  [radian]');
% tla = title('Heading Angle');
% set(tla,'FontName', 'Times new roman','fontsize',14)
% set(xla,'FontName', 'Times new roman','fontsize',12)
% set(yla,'FontName', 'Times new roman','fontsize',13)
% legend('\psi (PFRV)','\psi (PF)','fontsize',10)
% 
% subplot(414)
% plot(timee,del_n_his_3(:,1),'r','LineWidth',2)
% hold on
% plot(timee,del_n_his_2(:,1),'b:','LineWidth',2)
% plot(timee,del_n_his(:,1),'c:','LineWidth',2)
% grid minor
% % axis([0 50 0 1000])
% 
% xla = xlabel('Time [sec]');
% yla = ylabel('\delta_n  [rpm]');
% tla = title('Thruster RPM');
% set(tla,'FontName', 'Times new roman','fontsize',14)
% set(xla,'FontName', 'Times new roman','fontsize',12)
% set(yla,'FontName', 'Times new roman','fontsize',13)
% legend('\delta_n(PFRV)','\delta_n(PF)','\delta_n(Leader)','fontsize',10)

% f3=figure(3);
% plot(err_his_de_2(:,1),'r')
% hold on
% plot(psi_d_his_2(:,1),'b')
% plot(psi_his_2,'k')
% 
% f4=figure(4);
% aa = surf(Z_2)
% aa.EdgeColor = 'none';
% colorbar
% f3.Position = [300 300 540 400];
% f4=figure(4);
% quiver(-fx_3,-fy_3)
% f3.Position = [300 500 540 400];

% f3=figure(3)
% plot(V_d_his(3:length(u_2_his),1),'r')
% hold on
% plot(u_2_his(3:length(u_2_his),1),'b')

% f4=figure(4)
% subplot(211)
% plot(psi_d_his_2(3:length(u_2_his),1),'r')
% hold on
% plot(psi_his_2(3:length(u_2_his),1),'b')
% 
% subplot(212)
% plot(err_his_2(3:length(u_2_his),1),'r')
% grid minor

rms(err_his_2)
% 
% 
% f3=figure(4)
% plot(u_2_his(:,1))



% 
%% figure22
close all
f1=figure(1)
f1.Position = [100 100 540 400];

for i =1:5:length(local_his)
a(i) = plot(local_his(1:i,1),local_his(1:i,2),'b')
hold on;
a2(i)=plot(local_his_2(1:i,1),local_his_2(1:i,2),'r:','LineWidth',2);
hold on;
% quiver(-fx_2,-fy_2)
% aa.Color(4)=0.3;
a3(i)=plot(local_his_3(1:i,1),local_his_3(1:i,2),'k:','LineWidth',2);
a4(i)=plot(local_his_4(1:i,1),local_his_4(1:i,2),'c:','LineWidth',2);
a5(i)=plot(local_his_5(1:i,1),local_his_5(1:i,2),'g:','LineWidth',2);
dtt=1;
hold on
% grid minor
a6(i)=plot(trans_2_his(i,1),trans_2_his(i,2),'g*');
a7(i)=plot(trans_3_his(i,1),trans_3_his(i,2),'g*');
a8(i)=plot(trans_4_his(i,1),trans_4_his(i,2),'g*');
a9(i)=plot(trans_5_his(i,1),trans_5_his(i,2),'g*');



plot(x_2_trans_vec,y_2_trans_vec,'c*')
axis equal
%axis([220 265 7 42])
tit=title('Trajectory');
set(tit,'FontName', 'Times new roman', 'fontsize',20)
xla = xlabel('x (m)');
yla = ylabel('y (m)');
set(xla,'FontName', 'Times new roman','fontsize',20)
set(yla,'FontName', 'Times new roman','fontsize',20)
drawnow;
if i>1
    delete(a(i-1));
    delete(a2(i-1));
    delete(a3(i-1));
    delete(a4(i-1));
    delete(a5(i-1));
    delete(a6(i-1));
    delete(a7(i-1));
    delete(a8(i-1));
    delete(a9(i-1));
    delete(a6(i));    
    delete(a7(i));
    delete(a8(i));
    delete(a9(i));
end

end


