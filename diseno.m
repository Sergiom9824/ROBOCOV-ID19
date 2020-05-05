clear all, close all, clc

s=tf('s');

A=[0 1;0 -8.512];
B=[0;7689];
C=[1 0];
D=0;

[num,den]=ss2tf(A,B,C,D);

Gp=tf(num,den)

C1=[1 0;0 1];
D1=[0;0];

K=pidtune(Gp,'PID')
kp=0.0199;
kd=0.0029;
%ki=0;%K.Ki;

Kpos=kp+kd*s;  % PD para posición

CL=feedback(Gp*Kpos,1);

damp(CL)

figure, step(CL,10)

Gv=Gp*s;
Gv=minreal(Gv)

Kv=pidtune(Gv,'PI')

kp=0.01745;
ki=0.6572;

Kvel=kp+ki/s;  % PI para velocidad

CL=feedback(Gv*Kvel,1);

damp(CL)

figure, step(CL)











