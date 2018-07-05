clear all

folder = './psm12_6.30/';
testname = 'Take3';
csvname = strcat(folder, testname ,'.csv');

M = csvread(csvname,7,2);
%1: Rigid Body 1
%21: Rigid Body 2
%41: Rigid Body 3
%61: Rigid Body 4

%% Position PSMs
p1.quat = M(:,41:44); %xyzw 
p1.quat = [p1.quat(:,4),p1.quat(:,1:3)];
p1.rot  = quat2rotm(p1.quat);

p2.quat = M(:,61:64);
p2.quat = [p2.quat(:,4),p2.quat(:,1:3)];
p2.rot  = quat2rotm(p2.quat);

p1.pos = M(:,45:47);
p2.pos = M(:,65:67);

for i=1:length(p1.pos)
    p1.T(:,:,i) = [[p1.rot(:,:,i), p1.pos(i,:)'];[0,0,0,1]];
    p2.T(:,:,i) = [[p2.rot(:,:,i), p2.pos(i,:)'];[0,0,0,1]];
end

[p1.D, p1.res] = pivot(p1.T);
[p2.D, p2.res] = pivot(p2.T);

p1.rcm0 = p1.D(4:6);
p2.rcm0 = p2.D(4:6);

rel_p1 = p2.rcm0-p1.rcm0; %pos of p2 from p1: In camera frame, next put into PSM1 frame.

%% Rotations PSMs

r1.pos = M(:,5:7);
r1.rot = M(:,1:4);

r2.pos = M(:,25:27);
r2.rot = M(:,21:24);

r1.x = M(:,9:11); %rigid body 1_1
r1.y = M(:,13:15);%rigid body 1_2
r1.z = M(:,17:19);%rigid body 1_3

r2.x = M(:,29:31);
r2.y = M(:,33:35);
r2.z = M(:,37:39);

% ------------------------ %

r1 = get_rotation(r1);
r2 = get_rotation(r2);

R12 = r1.rot.'*r2.rot; 


%% Relative position of PSM1 to PSM2
P12_0 = p2.rcm0 - p1.rcm0;

P12_1 = r1.rot.'*P12_0;
%% Visualize





%% Answer
R12
P12_1


T12 = [[R12, P12_1];[0 0 0 1]];

T21 = inv(T12);

savename = strcat(folder,testname,'_T12.csv');
csvwrite(savename,T12);

savename = strcat(folder,testname,'_T21.csv');
csvwrite(savename,T21);




%% Functions

function [r] = get_rotation(r)

r.vz = mean(r.x-r.y);
r.vx = mean(r.z - r.y);
r.vy = cross(r.vz,r.vx);

r.vz = v_norm(r.vz);
r.vx = v_norm(r.vx);
r.vy = v_norm(r.vy);

r.rot = [r.vx.',r.vy.',r.vz.']; %R01

end

function [y] = v_norm(x)
y = x/norm(x);
end