function [Z_re,psi_d_re,point_vec_re,vertex_1] = potential_field_basic(ship,target,obs_1,vel_1,err_1,obs_2,vel_2,err_2,obs_3,vel_3,err_3)

% ship = [50 50];

% target= [150 150];

% obs_1 = [85 130];

% vel_1 = [1 1];

ship = round(ship);
% starting_point = round(ship);
obs_1 = round(obs_1);
vel_1 = vel_1;

obs_2 = round(obs_2);
vel_2 = vel_2;

obs_3 = round(obs_3);
vel_3 = vel_3;


obs_cir = 5;

a = 1; %%% sigmoid 함수의 최대 크기
b = 1; %%% sigmoid 함수의 기울기
k_vel = 10; %%% velocity gain

vertex_1 = [ship(2) + round(k_vel*vel_1(2)) , ship(1) + round(k_vel*vel_1(1))]; %%% cone 꼭지점
vertex_2 = [ship(2) + round(k_vel*vel_2(2)) , ship(1) + round(k_vel*vel_2(1))]; %%% cone 꼭지점
vertex_3 = [ship(2) + round(k_vel*vel_3(2)) , ship(1) + round(k_vel*vel_3(1))]; %%% cone 꼭지점

tangent_obs_1 = atan2(obs_1(2)-vertex_1(1),obs_1(1)-vertex_1(2))*180/3.1415; %%% cone의 기울기
tangent_obs_2 = atan2(obs_2(2)-vertex_2(1),obs_2(1)-vertex_2(2))*180/3.1415; %%% cone의 기울기
tangent_obs_3 = atan2(obs_3(2)-vertex_3(1),obs_3(1)-vertex_3(2))*180/3.1415; %%% cone의 기울기

dist_con_ver_1 =  sqrt((obs_1(2)-vertex_1(1))^2+(obs_1(1)-vertex_1(2))^2); %%% vertex와 장애물 간의 거리
dist_con_ver_2 =  sqrt((obs_2(2)-vertex_2(1))^2+(obs_2(1)-vertex_2(2))^2); %%% vertex와 장애물 간의 거리
dist_con_ver_3 =  sqrt((obs_3(2)-vertex_3(1))^2+(obs_3(1)-vertex_3(2))^2); %%% vertex와 장애물 간의 거리

tangent_cone_1_1 = real(tangent_obs_1 - asin(obs_cir/dist_con_ver_1)*180/3.1415);  %%% 접선의 기울기 1
tangent_cone_1_2 = real(tangent_obs_1 + asin(obs_cir/dist_con_ver_1)*180/3.1415);  %%% 접선의 기울기 2
tangent_cone_2_1 = real(tangent_obs_2 - asin(obs_cir/dist_con_ver_2)*180/3.1415);  %%% 접선의 기울기 1
tangent_cone_2_2 = real(tangent_obs_2 + asin(obs_cir/dist_con_ver_2)*180/3.1415);  %%% 접선의 기울기 2
tangent_cone_3_1 = real(tangent_obs_3 - asin(obs_cir/dist_con_ver_3)*180/3.1415);  %%% 접선의 기울기 1
tangent_cone_3_2 = real(tangent_obs_3 + asin(obs_cir/dist_con_ver_3)*180/3.1415);  %%% 접선의 기울기 2

obs1_angle_1 = min(tangent_cone_1_1,tangent_cone_1_2);
obs1_angle_2 = max(tangent_cone_1_1,tangent_cone_1_2);
obs2_angle_1 = min(tangent_cone_2_1,tangent_cone_2_2);
obs2_angle_2 = max(tangent_cone_2_1,tangent_cone_2_2);
obs3_angle_1 = min(tangent_cone_3_1,tangent_cone_3_2);
obs3_angle_2 = max(tangent_cone_3_1,tangent_cone_3_2);

Z=zeros(300);
Z_1=zeros(300); %%% obs map 초기화
Z_2=zeros(300);
Z_3=zeros(300);
Z_4=zeros(300); %%% target map

dt = 1;
dtt = 1;
i=1;

for t=1:dt:299    %%% field 계산
    ii=1;
    for tt=1:dtt:299
       Z_1(ii,i) = 2*tanh(1*(1/(real(sqrt((obs_1(1)-i)^2+(obs_1(2)-ii)^2)))));
       if Z_1(ii,i) < 0 || abs(err_1) < 15
           Z_1(ii,i) = 0;
       end
       Z_2(ii,i) = 2*tanh(1*(1/(real(sqrt((obs_2(1)-i)^2+(obs_2(2)-ii)^2)))));
       if Z_2(ii,i) < 0 || abs(err_2) < 15
           Z_2(ii,i) = 0;
       end
       Z_3(ii,i) = 2*tanh(1*(1/(real(sqrt((obs_3(1)-i)^2+(obs_3(2)-ii)^2)))));
       if Z_3(ii,i) < 0 || abs(err_3) < 15
           Z_3(ii,i) = 0;
       end
       Z_4(ii,i) = 1*tanh(35*(1/(real(sqrt((ship(1)-i)^2+(ship(2)-ii)^2)))))-1*tanh(35*(1/(real(sqrt((target(1)-i)^2+(target(2)-ii)^2)))));
       A = [Z_1(ii,i) Z_2(ii,i) Z_3(ii,i) Z_4(ii,i)];
       
       Z(ii,i) = max(A);
       if Z_4(ii,i) < 0
           Z(ii,i) = Z(ii,i) + Z_4(ii,i);
       end
       Z(ii,i) = Z(ii,i);
       ii=ii+1;
    end
    i=i+1;
end
[fx,fy] = gradient(Z,1);
if ship(1) < 1
    ship(1)=1;
end

if ship(2) < 1
    ship(2)=1;
end
point_vec = -[fx(ship(2),ship(1)),fy(ship(2),ship(1))];
psi_d = atan2(point_vec(2),point_vec(1))*180/3.1415;

% Z_crop = Z(ship(2) - 2:ship(2) + 2,ship(1) - 2:ship(1) + 2);
% 
Z_re = Z;
% [fx_c,fy_c] = gradient(Z_crop,1);
% point_vec_2 = -[fx_c(3,3),fy_c(3,3)];
% psi_d_crop = atan2(point_vec_2(2),point_vec_2(1))*180/3.1415;
psi_d_re = psi_d;
point_vec_re=point_vec;

end


