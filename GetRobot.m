function robot = GetRobot()

dhParams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];

robot = robotics.RigidBodyTree();

body1 = robotics.RigidBody('body1');
body2 = robotics.RigidBody('body2');
body3 = robotics.RigidBody('body3');
body4 = robotics.RigidBody('body4');
body5 = robotics.RigidBody('body5');
body6 = robotics.RigidBody('body6');

jnt1 = robotics.Joint('jnt1','revolute');
jnt2 = robotics.Joint('jnt2','revolute');
jnt3 = robotics.Joint('jnt3','revolute');
jnt4 = robotics.Joint('jnt4','revolute');
jnt5 = robotics.Joint('jnt5','revolute');
jnt6 = robotics.Joint('jnt6','revolute');


setFixedTransform(jnt1,dhParams(1,:),'dh');
setFixedTransform(jnt2,dhParams(2,:),'dh');
setFixedTransform(jnt3,dhParams(3,:),'dh');
setFixedTransform(jnt4,dhParams(4,:),'dh');
setFixedTransform(jnt5,dhParams(5,:),'dh');
setFixedTransform(jnt6,dhParams(6,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body1,'base');
addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
addBody(robot,body4,'body3');
addBody(robot,body5,'body4');
addBody(robot,body6,'body5');
end