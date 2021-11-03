rosinit
global odom

sub_odom = rossubscriber("/odom",@odom_callback);

[pub_q1,msg_q1] = rospublisher('/mobile_manipulator/base_joint_position/command','std_msgs/Float64');
[pub_q2,msg_q2] = rospublisher('/mobile_manipulator/link_1_joint_position/command','std_msgs/Float64');
[pub_q3,msg_q3] = rospublisher('/mobile_manipulator/link_2_joint_position/command','std_msgs/Float64');
[pub_q4,msg_q4] = rospublisher('/mobile_manipulator/link_3_joint_position/command','std_msgs/Float64');
[pub_vel,msg_vel] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%scandata = rosmessage("std_msgs/Float64")
q_temp = [0,0,0,0];

msg_vel.Linear.X = 0.5;
msg_vel.Angular.Z = -0.5;

rate = robotics.Rate(1);
while rate.TotalElapsedTime < 10
    odom.Pose.Pose.Position
    
    if q_temp(2) == qi(2)
        q_temp = [0,0,0,0];
    else
        q_temp = qi;
    end
    
    msg_q1.Data = q_temp(1);
    msg_q2.Data = q_temp(2);
    msg_q3.Data = q_temp(3);
    msg_q4.Data = q_temp(4);
    
    send(pub_q1,msg_q1)
    send(pub_q2,msg_q2)
    send(pub_q3,msg_q3)
    send(pub_q4,msg_q4) 
    
    send(pub_vel,msg_vel)
    
    waitfor(rate);
end

msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)

rosshutdown

function odom_callback(src,msg)
    global odom
    odom = msg; 
end