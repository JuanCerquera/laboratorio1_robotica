L1 = 2.45;
L2 = 3.78;
L3 = 7.10;
L4 = 3.00;
L5 = 5.40;
L6 = 1.50;
L7 = 1.60;

%% Toolbox de Peter Corke

L(1) = Link('revolute','a',0,'alpha',0,'d',L1,'offset',0,'modified','qlim',[-pi,pi]);
L(2) = Link('revolute','a',0,'alpha',pi/2,'d',-L2,'offset',pi/2,'modified','qlim',[-pi,pi]);
L(3) = Link('revolute','a',L3,'alpha',pi,'d',-L4,'offset',0,'modified','qlim',[-3*pi/2,3*pi/2]);
L(4) = Link('revolute','a',0,'alpha',-pi/2,'d',-L5,'offset',0,'modified','qlim',[-pi,pi]);
L(5) = Link('revolute','a',0,'alpha',pi/2,'d',L6,'offset',0,'modified','qlim',[-pi,pi]);
L(6) = Link('revolute','a',0,'alpha',-pi/2,'d',0,'offset',0,'modified','qlim',[-pi,pi]);

% Cinemática directa
syms q1 q2 q3 q4 q5 q6
digits(4)
A_0_1 = L(1).A(q1);
A_1_2 = L(2).A(q2);
A_2_3 = L(3).A(q3);
A_3_4 = L(4).A(q4);
A_4_5 = L(5).A(q5);
A_5_6 = L(6).A(q6);

A_tool = transl(0,0,-L7)*round(trotx(pi),2);

T_0_tcp = simplify(A_0_1*A_1_2*A_2_3*A_3_4*A_4_5*A_5_6*A_tool);
T_0_tcp = simplify(T_0_tcp);
R = simplify(T_0_tcp(1:3,1:3));

for i = 1:4
    for j = 1:4
        [C,T] = coeffs(T_0_tcp(i,j));
        C = round(double(C),2);
        T_0_tcp(i,j) = dot(C,T);
        display([i,j])
        T_0_tcp(i,j)
    end
end
%%

robot_1 = SerialLink(L,'name','CRX-10iA');
robot_1.tool = A_tool;
figure()
%robot_1.fkine([0 0 0 0 0 0]);
robot_1.plot([0,0,0,0,0,0],'workspace',[-10 10 -10 10 -1 12],'noa', 'view',[50 50])
%robot_1.teach([0 0 0 0 0 0])

%% Robotics System Toolbox
%Eslabones
body1 = rigidBody('body1');
body2 = rigidBody('body2');
body3 = rigidBody('body3');
body4 = rigidBody('body4');
body5 = rigidBody('body5');
body6 = rigidBody('body6');

% Articulaciones
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = pi/2;
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = 0;
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.HomePosition = 0;
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.HomePosition = 0;
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.HomePosition = 0;

% Transformaciones fijas
tform1 = axang2tform([1 0 0 0])*trvec2tform([0,0,0])*trvec2tform([0,0,L1]);
tform2 = axang2tform([1 0 0 pi/2])*trvec2tform([0,0,0])*trvec2tform([0,0,-L2]);
tform3 = axang2tform([1 0 0 pi])*trvec2tform([L3,0,0])*trvec2tform([0,0,-L4]);
tform4 = axang2tform([1 0 0 -pi/2])*trvec2tform([0,0,0])*trvec2tform([0,0,-L5]);
tform5 = axang2tform([1 0 0 pi/2])*trvec2tform([0,0,0])*trvec2tform([0,0,L6]);
tform6 = axang2tform([1 0 0 -pi/2])*trvec2tform([0,0,0])*trvec2tform([0,0,0]);

setFixedTransform(jnt1,tform1);
setFixedTransform(jnt2,tform2);
setFixedTransform(jnt3,tform3);
setFixedTransform(jnt4,tform4);
setFixedTransform(jnt5,tform5);
setFixedTransform(jnt6,tform6);

% Asignación de eslabones con articulaciones
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

% Creación del robot
robot_2 = rigidBodyTree;

% Añadimos los eslabones
addBody(robot_2,body1,'base');
addBody(robot_2,body2,'body1');
addBody(robot_2,body3,'body2');
addBody(robot_2,body4,'body3');
addBody(robot_2,body5,'body4');
addBody(robot_2,body6,'body5');

% Herramienta
tool = rigidBody('tool');
tform7 = axang2tform([1 0 0 pi])*trvec2tform([0,0,L7]);
setFixedTransform(tool.Joint,tform7);
addBody(robot_2,tool,'body6');

% Mostramos robot
figure()
show(robot_2)

%% Cinemática directa
JointNames = ["jnt1" "jnt2" "jnt3" "jnt4" "jnt5" "jnt6"];
JointPositions = [0 pi/2 0 0 0 0];
for i=1:6
    config(i).JointName = JointNames(i);
    config(i).JointPosition = JointPositions(i);
end

GeomDir1 = robot_1.fkine([0 0 0 0 0 0])
GeomDir2 = getTransform(robot_2,config,'tool')


%% Coordenadas generalizadas
x = GeomDir1(1,4)
y = GeomDir1(2,4)
z = GeomDir1(3,4)
rpy = tr2rpy(GeomDir1);
theta_x = rpy(1)
theta_y = rpy(2)
theta_z = rpy(3)
