clear;clc;
% 速度直接控制器 PD
% 被控系统 1/s

kp=150;
kd=20;
G=tf([kd,kp],[kd+1,kp]);
bode(G)