% rosinit;
[pub,msg] = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
msg.Linear.X = 0.5;
msg.Angular.Z = -0.5;
rate = robotics.Rate(10);
while rate.TotalElapsedTime < 100
   send(pub,msg)
   waitfor(rate);
end

%rosshutdown
