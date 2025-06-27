function T = trans_cal(alpha_i,a_i,d_i,theta_i)
%计算变换矩阵函数T_{i-1,i}
%输入的参数为，alpha_{i},a_{i},d_i,theta_i,与DH表达法的参数表对应
%注意，这里输入的角度，均采用角度制，不采用弧度制

T = [cosd(theta_i)     -sind(theta_i)  0   a_i
     sind(theta_i)*cosd(alpha_i)     cosd(alpha_i)*cosd(theta_i)   -sind(alpha_i) -sind(alpha_i)*d_i
     sind(theta_i)*sind(alpha_i)     cosd(theta_i)*sind(alpha_i)   cosd(alpha_i)  cosd(alpha_i)*d_i
     0                             0                           0                         1];
end