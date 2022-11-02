close all
import ETS3.*

j1 = Revolute('d', L2, 'a', 0, 'alpha', -pi/2, 'offset', 0);
j2 = Revolute('d', 0, 'a', -L3, 'alpha', 0, 'offset', pi/2);
j3 = Revolute('d', 0, 'a', -L4, 'alpha', 0, 'offset', 0);
j4 = Revolute('d', 0, 'a', -L5, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2 j3 j4],'name', 'my robot');

robot.plot([0,0,0,0])
