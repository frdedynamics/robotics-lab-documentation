close all
import ETS3.*

L1=0;
L2=0.2;
L3=0.385;
L4=0.270;
L5=0.115;

robot_normal = Tz(L1) * Rz('q1') * Tz(L2) * Ry('q2') * Tz(L3) * Ry('q3') * Tz(L4) * Ry('q4') * Tz(L5);
robot_normal.plot([0,0,0,0])