clear;clc;
% 位置直接控制器 PD
% 被控系统 1/s^2
% 带宽\sqrt{2kp}, 相位裕度 
kp=150;
kd=20;
G=tf([kd,kp],[1,kd,kp]);
bode(G)