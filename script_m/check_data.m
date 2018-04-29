clear all
rosshutdown
rosinit

setenv('ROS_MASTER_URI','http://aimlab2:11311')

sub = rossubscriber('/dvrk/PSM2/jacobian_body');
msg = receive(sub,10);

%% Jacobian Body
J_b = msg.Data;


sub = rossubscriber('/dvrk/PSM2/jacobian_spatial');
msg = receive(sub,10);

%% Jacobian Spatial
J_spatial = msg.Data;

