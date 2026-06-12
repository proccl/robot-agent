function robotTree = buildRobotTree(arm)
% buildRobotTree 將 Arm7R 的 DH/變換參數轉換為 rigidBodyTree
%   arm: Arm7R 對象
%   robotTree: rigidBodyTree 對象，運動學與 Arm7R 完全一致
%
%   為了精確匹配 Arm7R 的 forwardKinematics，根據每個關節「旋轉矩陣與固定
%   變換的相對順序」決定 rigidBodyTree 的組裝方式：
%     • 旋轉在固定變換之後：dummy fixed body（放固定變換）+ revolute body
%     • 旋轉在固定變換之前：revolute dummy body + fixed body（放固定變換）
%   由此得到 14 個 body（base 不算），7 個非固定關節，配置向量順序即 q1..q7。

    if nargin < 1 || isempty(arm)
        arm = Arm7R();
    end
    
    e = arm.e;
    k = arm.k;
    i = arm.i;
    l = arm.l;
    m = arm.m;
    n = arm.n;
    j = arm.j;
    b = arm.b;
    
    robotTree = rigidBodyTree('DataFormat', 'row');
    
    %% Joint 1: rotation after fixed transform
    % T01(q) = T01_0 * roty(q)
    T01_0 = [0, 0, 1, 0;
             1, 0, 0, 0;
             0, 1, 0, e;
             0, 0, 0, 1];
    
    dummy1 = rigidBody('dummy1');
    j1f = rigidBodyJoint('j1_fixed', 'fixed');
    setFixedTransform(j1f, T01_0);
    dummy1.Joint = j1f;
    addBody(robotTree, dummy1, 'base');
    
    body1 = rigidBody('body1');
    j1 = rigidBodyJoint('j1', 'revolute');
    j1.JointAxis = [0; 1; 0];
    j1.PositionLimits = [arm.q_limit_lower(1), arm.q_limit_upper(1)];
    body1.Joint = j1;
    addBody(robotTree, body1, 'dummy1');
    
    %% Joint 2: rotation after fixed transform
    % T12(q) = T12_0 * roty(q)
    T12_0 = [0, 0, 1, 0;
             1, 0, 0, 0;
             0, 1, 0, k;
             0, 0, 0, 1];
    
    dummy2 = rigidBody('dummy2');
    j2f = rigidBodyJoint('j2_fixed', 'fixed');
    setFixedTransform(j2f, T12_0);
    dummy2.Joint = j2f;
    addBody(robotTree, dummy2, 'body1');
    
    body2 = rigidBody('body2');
    j2 = rigidBodyJoint('j2', 'revolute');
    j2.JointAxis = [0; 1; 0];
    j2.PositionLimits = [arm.q_limit_lower(2), arm.q_limit_upper(2)];
    body2.Joint = j2;
    addBody(robotTree, body2, 'dummy2');
    
    %% Joint 3: rotation before fixed transform
    % T23(q) = rotz(q) * trans([l; 0; i])
    dummy3 = rigidBody('dummy3');
    j3 = rigidBodyJoint('j3', 'revolute');
    j3.JointAxis = [0; 0; 1];
    j3.PositionLimits = [arm.q_limit_lower(3), arm.q_limit_upper(3)];
    dummy3.Joint = j3;
    addBody(robotTree, dummy3, 'body2');
    
    body3 = rigidBody('body3');
    j3f = rigidBodyJoint('j3_fixed', 'fixed');
    T23_0 = eye(4);
    T23_0(1:3, 4) = [l; 0; i];
    setFixedTransform(j3f, T23_0);
    body3.Joint = j3f;
    addBody(robotTree, body3, 'dummy3');
    
    %% Joint 4: rotation before fixed transform
    % T34(q) = rotz(q) * trans([n; 0; -m])
    dummy4 = rigidBody('dummy4');
    j4 = rigidBodyJoint('j4', 'revolute');
    j4.JointAxis = [0; 0; 1];
    j4.PositionLimits = [arm.q_limit_lower(4), arm.q_limit_upper(4)];
    dummy4.Joint = j4;
    addBody(robotTree, dummy4, 'body3');
    
    body4 = rigidBody('body4');
    j4f = rigidBodyJoint('j4_fixed', 'fixed');
    T34_0 = eye(4);
    T34_0(1:3, 4) = [n; 0; -m];
    setFixedTransform(j4f, T34_0);
    body4.Joint = j4f;
    addBody(robotTree, body4, 'dummy4');
    
    %% Joint 5: rotation after fixed transform
    % T45(q) = T45_0 * roty(-q)
    T45_0 = [1, 0, 0, 0;
             0, 0, 1, 0;
             0, -1, 0, 0;
             0, 0, 0, 1];
    
    dummy5 = rigidBody('dummy5');
    j5f = rigidBodyJoint('j5_fixed', 'fixed');
    setFixedTransform(j5f, T45_0);
    dummy5.Joint = j5f;
    addBody(robotTree, dummy5, 'body4');
    
    body5 = rigidBody('body5');
    j5 = rigidBodyJoint('j5', 'revolute');
    j5.JointAxis = [0; -1; 0];
    j5.PositionLimits = [arm.q_limit_lower(5), arm.q_limit_upper(5)];
    body5.Joint = j5;
    addBody(robotTree, body5, 'dummy5');
    
    %% Joint 6: rotation after fixed transform
    % T56(q) = T56_0 * roty(-q)
    T56_0 = [0, 0, 1, 0;
             -1, 0, 0, 0;
             0, -1, 0, j;
             0, 0, 0, 1];
    
    dummy6 = rigidBody('dummy6');
    j6f = rigidBodyJoint('j6_fixed', 'fixed');
    setFixedTransform(j6f, T56_0);
    dummy6.Joint = j6f;
    addBody(robotTree, dummy6, 'body5');
    
    body6 = rigidBody('body6');
    j6 = rigidBodyJoint('j6', 'revolute');
    j6.JointAxis = [0; -1; 0];
    j6.PositionLimits = [arm.q_limit_lower(6), arm.q_limit_upper(6)];
    body6.Joint = j6;
    addBody(robotTree, body6, 'dummy6');
    
    %% Joint 7: rotation after fixed transform
    % T67(q) = T67_0 * rotz(-q)
    T67_0 = [1, 0, 0, 0;
             0, -1, 0, 0;
             0, 0, -1, -b;
             0, 0, 0, 1];
    
    dummy7 = rigidBody('dummy7');
    j7f = rigidBodyJoint('j7_fixed', 'fixed');
    setFixedTransform(j7f, T67_0);
    dummy7.Joint = j7f;
    addBody(robotTree, dummy7, 'body6');
    
    body7 = rigidBody('body7');
    j7 = rigidBodyJoint('j7', 'revolute');
    j7.JointAxis = [0; 0; -1];
    j7.PositionLimits = [arm.q_limit_lower(7), arm.q_limit_upper(7)];
    body7.Joint = j7;
    addBody(robotTree, body7, 'dummy7');
    
    % 為關鍵 body 添加簡化碰撞幾何（球體近似），用於末端/關節點干涉檢測
    addJointSphereCollisions(robotTree);
end

function addJointSphereCollisions(tree)
% addJointSphereCollisions 為每個非固定 body 的原點添加小球碰撞體
%   這是一種簡化近似，可檢測關節點進入障礙物的情況；
%   未來可替換為更精確的連桿圓柱體碰撞體。

    sphere_radius = 20;  % mm
    for k = 1:tree.NumBodies
        body = tree.Bodies{k};
        if ~strcmp(body.Joint.Type, 'fixed')
            addCollision(body, collisionSphere(sphere_radius));
        end
    end
end
