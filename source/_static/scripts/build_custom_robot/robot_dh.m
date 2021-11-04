close all
import ETS3.*

j = Revolute('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2 j3 j4],'name', 'my robot');

robot.plot([0,0,0,0])
