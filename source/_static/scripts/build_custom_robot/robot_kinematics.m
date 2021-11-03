T_robot_1 = robot.fkine([0,pi/2,pi/2,0])
T_robot_2 = SE3(0.385,0,-0.185) * SE3.rpy(0,0,90, 'deg')
qi = robot.ikine(T_robot_1, 'mask', [1 1 1 0 0 0])

robot.plot(qi)