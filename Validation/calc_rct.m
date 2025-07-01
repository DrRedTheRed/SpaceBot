q = [0 pi/2 0 0 0 -pi/2 0]; %由于toolbox抽羊癫疯认为当关节为旋转关节时，theta所见
% 即所得，因此只好这样指定初始偏移值。

scale = 1000;%为了画图的缩放

% [a      alpha      d       theta]
mdhparams = [
    0         0     120      0;
    0      pi/2    -100      0;
    0      -pi/2    100      0;
    400     0       100      0;
   -400     0       100      0;
    0       pi/2    100      0;
    0       pi/2    120      0;
];

robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies', 8);

prevBody = robot.BaseName;

for i = 1:size(mdhparams, 1)
    body = robotics.RigidBody(['link' num2str(i)]);
    joint = robotics.Joint(['joint' num2str(i)], 'revolute');
    
    % 直接传入 [theta, alpha, a, d]，标准 D-H 顺序
    a = mdhparams(i, 1) / scale;
    alpha = mdhparams(i, 2);
    d     = mdhparams(i, 3) / scale;
    theta = mdhparams(i, 4);

    % setFixedTransform 使用改进 D-H 约定
    setFixedTransform(joint, [a, alpha, d, theta], 'mdh');
    
    body.Joint = joint;
    addBody(robot, body, prevBody);
    prevBody = body.Name;
end

% q = q + [180.0, 60.00000000000001, -49.49464967258142, -81.0107006548372, 130.5053503274186, -60.00000000000001, -180.0] * pi / 180;

q = q + [0.06671612,-0.06671593,0.02499931,-0.05000519,0.02500588,0.06671593,0.06671612];

% 显示机器人模型
showdetails(robot)

show(robot, q);