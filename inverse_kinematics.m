%%Inverse Kinematics
dhparams = [0 0 0 0; 0 0 0 0; 5.5 0 0 0; 5.5 0 0 0];
robot = rigidBodyTree;
bodies = cell(4,1);
joints = cell(4,1);
for i = 1:4
    bodies{i} = rigidBody(['body' num2str(i)]);
    if i == 1
        jntType = "prismatic";
    elseif i == 4
        jntType = "fixed";
    else
        jntType = "revolute";
    end
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], jntType);
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1
        addBody(robot,bodies{i},"base")
    else
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end
fprintf("done")
figure(Name="color robot")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor= 0.5);

ik = inverseKinematics('RigidBodyTree',robot)