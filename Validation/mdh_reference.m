syms x1 x2 x3 x4 x5 x6 x7 x y z A_X A_Y A_Z r11 r12 r13 r21 r22 r23 r31 r32 r33;
syms trans_matrix [4 4]
alpha = [0; 90; -90; 0; 0; 90; 90];
a   = [0; 0; 0; 400; -400; 0;0];
d   = [120, -100, -100, -100, -100, 100, 120];
theta0 = [0; 90; 0; 0; 0; -90; 0];

theta1 = [x1; x2; x3; x4; x5; x6; x7] + theta0;
test = trans(alpha(3:7),a(3:7),d(3:7),theta1(3:7));
simplify(test)

target_pos = [x; y; z];
eul = [r11, r12, r13; r21, r22, r23; r31, r32, r33];
rotm = inv(trans(alpha(1:2),a(1:2),d(1:2),theta1(1:2))) * [eul, target_pos; 0, 0, 0, 1];
simplify(rotm)