clear all 
close all

foldername = 'data/test1/';
psm = 'PSM1';


p1 = [0.1353, 0.20372, -0.0308];
R1 = [0.5430, -0.8397, 0.0082;
            0.8397, 0.5430, 0.0142;
            -0.0161, 0.0031, 0.9998];


T21 = [R1,p1.'; 0 0 0 1];



filename = strcat(foldername,psm,'_xe.csv');
xe(:,:,1) = csvread(filename);

filename = strcat(foldername,psm,'_xd.csv');
xd(:,:,1) = csvread(filename);

filename = strcat(foldername,psm,'_xf.csv');
xf(:,:,1) = csvread(filename);

filename = strcat(foldername,psm,'_f.csv');
f(:,:,1) = csvread(filename);

psm = 'PSM2';

filename = strcat(foldername,psm,'_xe.csv');
xe(:,:,2) = csvread(filename);

filename = strcat(foldername,psm,'_xd.csv');
xd(:,:,2) = csvread(filename);

filename = strcat(foldername,psm,'_xf.csv');
xf(:,:,2) = csvread(filename);

filename = strcat(foldername,psm,'_f.csv');
f(:,:,2) = csvread(filename);

t = linspace(1,length(f(:,:,2)),length(f(:,:,2)))/600;
%Process
xf = xf +xd;

for i = 1:1:length(xe(:,1,1))
    xe(i,1:4,1) = (T21* [xe(i,1:3,1), 1].').';
    xd(i,1:4,1) = (T21* [xd(i,1:3,1), 1].').';
    xf(i,1:4,1) = (T21* [xf(i,1:3,1), 1].').'; 
end

figure()
subplot(1,2,2);
plot3(xe(:,1,2),xe(:,2,2),xe(:,3,2));
hold on
plot3(xd(:,1,2),xd(:,2,2),xd(:,3,2));
hold on
plot3(xf(:,1,2),xf(:,2,2),xf(:,3,2));
hold off
legend('PSM2 xe','PSM2 xd','PSM2 xf');
grid on
pbaspect([1 1 1])

subplot(1,2,1);
plot3(xe(:,1,1),xe(:,2,1),xe(:,3,1));
hold on
plot3(xd(:,1,1),xd(:,2,1),xd(:,3,1));
hold on
plot3(xf(:,1,1),xf(:,2,1),xf(:,3,1));
hold off

legend('PSM1 xe','PSM1 xd','PSM1 xf');
grid on
pbaspect([1 1 1])

figure()
plot(t,f(:,1,1)');


