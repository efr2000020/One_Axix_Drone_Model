
%%%%%%%%%%%%%%%%%%%%%Actuator Model  transfer function [(w)/pwm]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kdc=6.5 ;    % dc gain 
taw=0.15;     % taw of the system 
G1=tf(kdc,[taw 1])
%%%%%%%%%%%%%%%%%%%%%%second transfer function [(T)/(w)]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kp= 0.015;  % propper constant 
raw=1.225    % density of air 
D=0.254;      % propeller diameter 
w=2*pi/60 ;   
p=pi/2      

G2=tf([1],[((p*kp*raw*D^2)^1/3)*w])
%%%%%%%%%%%%%%%%%%%%Third transfer function between [(theta)/thurest force]%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J=0.74 ;  % mass momemt of interia for the total sysetm 
L=0.325 ;   % lenght to the center 
G3=tf(L,[J 0 0])
%%%%%%%%%%%%%%%%%%%%getting the open loob system tranfer function %%%%%%%%%%%%%%%%%%%%%%%%%%%
s1= series(G1,G2)
s2=series(s1,G3)
SS1=ss(s2)        % state space of the system with out pid controller 
figure(9)
margin(s2)
%%%%%%%%%%%%%%%%%%%%Time domain analysis  and frequency domain analysis of thE OPEN LOOP SYSTEM%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
subplot(2,1,1)
step(s2);
subplot(2,1,2)
impulse(s2);
stepinfo(s2);
figure(2)
subplot(2,1,1)
bode(s2);
subplot(2,1,2)
nyquist(s2);
figure(3)
rlocus(s2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Time domain analysis  and frequency domain analysis ofclosed loop without pid %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H1=tf(1,1)
F1=feedback(s2,H1)
figure(4)
subplot(2,1,1)
step(F1);
subplot(2,1,2)
impulse(F1);
stepinfo(F1);
figure(5)
subplot(2,1,1)
bode(F1);
subplot(2,1,2)
nyquist(F1);
figure(6)
rlocus(F1)


%%%%%%%%%%%%%%%%%%%%%Time domain analysis  and frequency domain analysis of the closed loop system with  PID controller%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Gc=pidtune(s2,'PID')
gnew=series(s2,Gc);
F2=feedback(gnew,H1)

ss2=ss(F2)  %state space of the system with out pid controller 
figure(7)
subplot(2,1,1)
step(F2);
subplot(2,1,2)
impulse(F2) ;
stepinfo(F2)
figure(8)
subplot(2,1,1)
nyquist(F2);
subplot(2,1,2)
bode(F2);
figure(9)
rlocus(F2);



