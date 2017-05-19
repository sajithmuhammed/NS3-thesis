function [q00x,v00x,q10x,v10x,q20x,v20x,q30x,v30x,q40x,v40x] = Platoon_test_3Vehicletest(j)

%%
%%
%clear all
%  close all
%% to know execution time
t_ex = cputime;
%%
persistent q00
persistent v00
persistent a00
persistent da00
persistent q10
persistent v10
persistent a10
persistent da10
persistent q20
persistent v20
persistent a20
persistent da20
persistent q30
persistent v30
persistent a30
persistent da30
persistent q40
persistent v40
persistent a40
persistent da40
persistent q0final
persistent v0final
persistent a0final
persistent da0final
persistent q1final
persistent v1final
persistent a1final
persistent da1final
persistent q2final
persistent v2final
persistent a2final
persistent da2final
persistent q3final
persistent v3final
persistent a3final
persistent da3final
persistent q4final
persistent v4final
persistent a4final
persistent da4final
persistent vref
h=0.002; % timestep for the lower loop, it should run every 2 ms

%%
t=0;
time=0;
h1=0.1;
N=1700;  % with N=700 gives bad behaviour, car crashes at t= 67 seconds.
%%
for j=1:N
%{
    vref=vref;
     if j<=100  
      vref = j*0.06;      
      elseif (j>100 && j<=200)
      vref=vref;
      elseif (j>200 && j<=300)
      vref=(j-1)*0.06/2; 
      elseif (j>300 && j<=700)
      vref=vref;
      elseif j>700
      vref=( 18 - (j-500)*0.06)/2;          
     end   
   %}
A=1.5;
f=0.2;   
time1(1,j)=(j-1)/10;
    if time1(1,j)>=0 && time1(1,j)<94.5
        x0(j)=(-A/f*cos(f*(time1(1,j)/2))+17.5);
    end
    if time1(1,j)>=94.5 && time1(1,j)<120
        x0(j)=x0(944);
    end
    if time1(1,j)>=120 && time1(1,j)<170
       % Emergency braking conditions
       if(time1(1,j)<=127.2)
            x0(j)=x0(j-1)-3.5*(0.1);
            if(x0(j)<0.0015)
                x0(j)=0;
            end
       else
            x0(j)=x0(j-1);
       end 
    end
    if time1(1,j)>=170
       x0(j)=x0(1699);
    end

    vref=x0(j);


    %% initial states: Leader
% the overall trajectories
if (j==1)%%initialization

q0final=0;
v0final=vref;
a0final=0;
da0final=0;

v00=vref; % initial velocity of the leader
q00=29; % initial position of the leader
a00=0;  %initial accel. of the leader
da00=0; % rate of change of accel. of the leader
%%
%% initial states: follower 1
% the overall trajectories
q1final=0;
v1final=vref;
a1final=0;
da1final=0;

v10=vref; % initial velocity of the follower
q10=22; % initial position of the follower
a10=0; % initial acceleration of the follower
da10=0; % rate of change of accel. of the follower
%% initial states: follower 2
% the overall trajectories
q2final=0;
v2final=vref;
a2final=0;
da2final =0;

v20=vref; % initial velocity of the follower
q20=15; % initial position of the follower
a20=0; % initial acceleration of the follower
da20=0; % rate of change of accel. of the follower
%% initial states: follower 3
% the overall trajectories
q3final=0;
v3final=vref;
a3final=0;
da3final =0;

v30=vref; % initial velocity of the follower
q30=8; % initial position of the follower
a30=0; % initial acceleration of the follower
da30=0; % rate of change of accel. of the follower
%% initial states: follower 4
% the overall trajectories
q4final=0;
v4final=vref;
a4final=0;
da4final =0;

v40=vref; % initial velocity of the follower
q40=1; % initial position of the follower
a40=0; % initial acceleration of the follower
da40=0; % rate of change of accel. of the follower
end
    %}
%% leader
q0i=q00;v0i=v00;a0i=a00;da0i=da00;
[q0f,v0f,a0f,da0f]=SpeedController_Leader(vref,q00,v00,a00,da00);

q0final=[q0final q0f];
v0final=[v0final v0f];
a0final=[a0final a0f];
da0final=[da0final da0f];
   
q00=q0f(end);
v00=v0f(end);
a00=a0f(end);
da00=da0f(end);

%% follower 1
if(1)
    %PF
v1f=v0i; 
a1f=a0i;
q1desired=q0i;
k=1;
%{
    %PFF
v1f=v0i; % target speed of the follower equals the current speed of the leader%%%%%%%%%%%%%%%%%
a1f=a0i;  % target acceleration of the follower equals the current acceleration of the leader%%%%%%%%%%%%%%%%%
q1desired=q0i;
k=2;

  %}
  %{
    %LF
v1f=v0i; 
a1f=a0i;
q1desired=q0i;
k=1;
  %}  
end
% desired gap1:
  desired_gap1(j)=(-(v1f-(v10))*0.5 +7)*k;
  desired_gap_0_1(j)=-((v0i-v10))*0.5 +7;
% A trajectory generating function is used to generate the velocity
% trajectory of the follower based on the initial&final values of velocity&acceleration&positon
% every 100 ms:

[v1,a1]=traj(t,t+0.1,v10,v1f,a10,a1f,h);
distanceError01=((q1desired-q10)-(desired_gap1(j)));
v1=v1+(distanceError01/0.1+(v1f-v10))/10;
if v1(17) < 0 || v1(51) < 0
v1(:)=0;
end

q1i=q10;v1i=v10;a1i=a10;da1i=da10;
[q1f,v1f,a1f,da1f]=  SpeedController_follower1(v1(17),v1(34),v1(51),q10,v10,a10,da10);
q10=q1f(end);
v10=v1f(end);
a10=a1f(end);
da10=da1f(end);

q1final=[q1final q1f];
v1final=[v1final v1f];
a1final=[a1final a1f];
da1final=[da1final da1f];



%% follower 2
if(1)
    %PF
v2f=v1i; 
a2f=a1i;
q2desired=q1i;
k=1;
%{
    %PFF
v2f=v0i; % target speed of the follower equals the current speed of the leader%%%%%%%%%%%%%%%%%
a2f=a0i;  % target acceleration of the follower equals the current acceleration of the leader%%%%%%%%%%%%%%%%%
q2desired=q0i;
k=2;

  %}
  %{
    %LF
v2f=v0i; 
a2f=a0i;
q2desired=q0i;
k=2;
  %}  
end
% desired gap1:
desired_gap2(j)=(-(v2f-(v20))*0.5 +7)*k;
desired_gap_1_2(j)=-((v1i-v20))*0.5 +7;
% A trajectory generating function is used to generate the velocity
% trajectory of the follower based on the initial&final values of velocity&acceleration&positon
% every 100 ms:

[v2,a2]=traj(t,t+0.1,v20,v2f,a20,a2f,h);
distanceError12=((q2desired-q20)-(desired_gap2(j)));
v2=v2+(distanceError12/0.1+(v2f-v20))/10;
if v2(17) < 0 || v2(51) < 0 
v2(:)=0;
end

q2i=q20;v2i=v20;a2i=a20;da2i=da20;
[q2f,v2f,a2f,da2f]=  SpeedController_follower2(v2(17),v2(34),v2(51),q20,v20,a20,da20);
q20=q2f(end);
v20=v2f(end);
a20=a2f(end);
da20=da2f(end);

q2final=[q2final q2f];
v2final=[v2final v2f];
a2final=[a2final a2f];
da2final=[da2final da2f];

j
%% follower 3
if(1)

    %PF
v3f=v2i; 
a3f=a2i;
q3desired=q2i;
k=1;
%{
    %PFF
v3f=v1i; % target speed of the follower equals the current speed of the leader%%%%%%%%%%%%%%%%%
a3f=a1i;  % target acceleration of the follower equals the current acceleration of the leader%%%%%%%%%%%%%%%%%
q3desired=q1i;
k=2;
  %}
  %{
    %LF
v3f=v0i; 
a3f=a0i;
q3desired=q0i;
k=3;
  %}  
end
% desired gap1:
desired_gap3(j)=(-(v3f-(v30))*0.5 +7)*k;
desired_gap_2_3(j)=-((v2i-v30))*0.5 +7;
% A trajectory generating function is used to generate the velocity
% trajectory of the follower based on the initial&final values of velocity&acceleration&positon
% every 100 ms:

[v3,a3]=traj(t,t+0.1,v30,v3f,a30,a3f,h);
distanceError23=((q3desired-q30)-(desired_gap3(j)));
v3=v3+(distanceError23/0.1+(v3f-v30))/10;
if v3(17) < 0  || v3(51) < 0
v3(:)=0;
end

q3i=q30;v3i=v30;a3i=a30;da3i=da30;
[q3f,v3f,a3f,da3f]=  SpeedController_follower1(v3(17),v3(34),v3(51),q30,v30,a30,da30);
q30=q3f(end);
v30=v3f(end);
a30=a3f(end);
da30=da3f(end);

q3final=[q3final q3f];
v3final=[v3final v3f];
a3final=[a3final a3f];
da3final=[da3final da3f];


%% follower 4
if(1)
    %PF
v4f=v3i; 
a4f=a3i;
q4desired=q3i;
k=1;
    %{
    %PFF
v4f=v2i; % target speed of the follower equals the current speed of the leader%%%%%%%%%%%%%%%%%
a4f=a2i;  % target acceleration of the follower equals the current acceleration of the leader%%%%%%%%%%%%%%%%%
q4desired=q2i;
k=2;
%}
    %{
    %LF
v4f=v0i; 
a4f=a0i;
q4desired=q0i;
k=4;
  %}  
end
% desired gap1:
desired_gap4(j)=(-(v4f-(v40))*0.5+7)*k;
desired_gap_3_4(j)=(-(v3i-v40))*0.5 +7;
% A trajectory generating function is used to generate the velocity
% trajectory of the follower based on the initial&final values of velocity&acceleration&positon
% every 100 ms:

[v4,a4]=traj(t,t+0.1,v40,v4f,a40,a4f,h);
distanceError34=((q4desired-q40)-(desired_gap4(j)));
v4=v4+(distanceError34/0.1+(v4f-v40))/10;
if v4(17) < 0 || v4(51) < 0
v4(:)=0;
end

q4i=q40;v4i=v40;a4i=a40;da4i=da40;
[q4f,v4f,a4f,da4f]=  SpeedController_follower2(v4(17),v4(34),v4(51),q40,v40,a40,da40);
q40=q4f(end);
v40=v4f(end);
a40=a4f(end);
da40=da4f(end);

q4final=[q4final q4f];
v4final=[v4final v4f];
a4final=[a4final a4f];
da4final=[da4final da4f];


%%
t=t+0.1;
  time=[time time(length(time)):0.002:time(length(time))+0.1];
q00x=q00;
v00x=v00;
q10x=q10;
v10x=v10;
q20x=q20;
v20x=v20;
q30x=q30;
v30x=v30;
q40x=q40;
v40x=v40;
end
%% plot together:
figure
plot(time,q0final)
hold on
plot(time,q1final);
plot(time,q2final);
plot(time,q3final);
plot(time,q4final);
legend('leader','follower1','follower2','follower3','follower4');title('position')

figure
plot(time,v0final)
hold on
plot(time,v1final);
plot(time,v2final);
plot(time,v3final);
plot(time,v4final);
legend('leader','follower1','follower2','follower3','follower4');title('velocity')

figure
plot(time,a0final)
hold on
plot(time,a1final);
plot(time,a2final);
plot(time,a3final);
plot(time,a4final);
legend('leader','follower1','follower2','follower3','follower4');title('acceleration')

% difference:
figure
plot(time,v0final-v1final);title('error in velocity between vehicle 0 and vehicle 1');

figure
plot(time,v1final-v2final);title('error in velocity between vehicle 1 and vehicle 2');

figure
plot(time,v2final-v3final);title('error in velocity between vehicle 2 and vehicle 3');

figure
plot(time,v3final-v4final);title('error in velocity between vehicle 3 and vehicle 4');

figure()
plot(time,q0final-q1final);
hold on 
x1=linspace(0,N,length(time));
yy1=spline(1:N,desired_gap_0_1,x1);
plot(time,yy1);legend('difference between positions vehicle 0 and vehicle 1','desired gap')

figure()
plot(time,q1final-q2final);
hold on 
x2=linspace(0,N,length(time));
yy2=spline(1:N,desired_gap_1_2,x2);
plot(time,yy2);legend('difference between positions vehicle 1 and vehicle 2','desired gap')

figure()
plot(time,q2final-q3final);
hold on 
x3=linspace(0,N,length(time));
yy3=spline(1:N,desired_gap_2_3,x3);
plot(time,yy3);legend('difference between positions vehicle 2 and vehicle 3','desired gap')

figure()
plot(time,q3final-q4final);
hold on 
x4=linspace(0,N,length(time));
yy4=spline(1:N,desired_gap_3_4,x4);
plot(time,yy4);legend('difference between positions vehicle 3 and vehicle 4','desired gap')

%error
figure()
plot(time,q0final-q1final-yy1);title('error in position vehicle 0 and vehicle 1');

figure()
plot(time,q1final-q2final-yy2);title('error in position vehicle 1 and vehicle 2');

figure()
plot(time,q2final-q3final-yy3);title('error in position vehicle 2 and vehicle 3');

figure()
plot(time,q3final-q4final-yy4);title('error in position vehicle 3 and vehicle 4');

e = cputime-t_ex;

end

%%
function [qf,vf,af,daf]=SpeedController_Leader(w_des,q0,v0,a0,da0)

format short;
global A B C K F ;

%% parameters
tau = 0.2;
taua = 0.2;
Kt = 100;
Kta = 10;

%% reference
% w_des = 20;

%% System you want to control
A=[0          1               0;
   0          0               1;              
   0    -1/(tau*taua)   -(tau + taua)/(tau*taua); ];
B=[0; 0; (Kt*Kta)/(tau*taua);];
C=[1 0 0];



%% cntinuous-time poles 
eigs(A);
%% continuous-time state space
sys_cont = ss(A,B,C,0);

%% Sampling period
h = 0.002;

%% ZOH based discrete system
sys_disc= c2d(sys_cont, h);
phi = sys_disc.a;
Gamma = sys_disc.b;


%% controllability test
gamma = [Gamma phi*Gamma   phi^2*Gamma ];
det(gamma);


% if det(gamma) == 0
%     disp('Uncontrollable');
% else
%     disp('Controllable');
% end

 
%% discrete-time poles 
eigs(phi);

%% Desired closed-loop poles
alpha = [0.99 0.99  0.99];

%% feedback gain
K = -acker(phi,Gamma,alpha);
% [G,S,e] = dlqr(phi,Gamma,eye(3),1);
% K = -G;

%% feedforward gain
F = 1/(C*inv(eye(3)-phi-Gamma*K)*Gamma);

position(1)=q0;
x1(1) = v0;
x2(1) = a0;
x3(1) = da0;
input(1) = 0;
time(2) = h; time(1) = 0;

for i=1:0.1/h
    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des;
   %     u = 0;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
    x1(i+1) = xkp1(1);
   
%     % saturation level for the acceleration 
%     if xkp1(2)>4
%         x2(i+1) = 4;
%     elseif xkp1(2)<-4
%          x2(i+1) = -4;
%     else
%          x2(i+1) = xkp1(2);
%     end
     x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);

    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;

end
qf=position;
vf=x1;
af=x2;
daf=x3;
if abs(max(input))>12
 max(input)
end
end

%%
function [qf,vf,af,daf]=SpeedController_follower1(w_des1,w_des2,w_des3,q0,v0,a0,da0)

format short;
global A B C K F ;

%% parameters
tau = 0.3;
taua = 0.3;
Kt = 100;
Kta = 15;

%% reference
% w_des1 = 20;

%% System you want to control
A=[0          1               0;
   0          0               1;              
   0    -1/(tau*taua)   -(tau + taua)/(tau*taua); ];
B=[0; 0; (Kt*Kta)/(tau*taua);];
C=[1 0 0];
%% cntinuous-time poles 
eigs(A);
%% continuous-time state space
sys_cont = ss(A,B,C,0);
%% Sampling period
 h = 0.002;
%% ZOH based discrete system
sys_disc= c2d(sys_cont, h);
phi = sys_disc.a;
Gamma = sys_disc.b;


%% controllability test
gamma = [Gamma phi*Gamma   phi^2*Gamma ];
det(gamma);


% if det(gamma) == 0
%     disp('Uncontrollable');
% else
%     disp('Controllable');
% end

 
%% discrete-time poles 
eigs(phi);

%% Desired closed-loop poles
alpha = [0.99 0.99  0.99];

%% feedback gain
K = -acker(phi,Gamma,alpha);
% [G,S,e] = dlqr(phi,Gamma,eye(3),1);
% K = -G;

%% feedforward gain
F = 1/(C*inv(eye(3)-phi-Gamma*K)*Gamma);

position(1)=q0; 
x1(1) = v0;
x2(1) = a0;
x3(1) = da0;
input(1) = 0;
time(2) = h; time(1) = 0;

for i=1:16
    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des1;
   %     u = 0;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
        
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;
    
end
for i=17:32
    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des2;
   %     u = 0;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
        
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;
    
end
for i=33:50    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des3;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;    
end
    
    qf=position;
    vf=x1;
    af=x2;
    daf=x3;
    
if abs(max(input))>12
 max(input)
end

end

function [qf,vf,af,daf]=SpeedController_follower2(w_des1,w_des2,w_des3,q0,v0,a0,da0)

format short;
global A B C K F ;

%% parameters
tau = 0.2;
taua = 0.2;
Kt = 100;
Kta = 10;

%% reference
% w_des = 20;

%% System you want to control
A=[0          1               0;
   0          0               1;              
   0    -1/(tau*taua)   -(tau + taua)/(tau*taua); ];
B=[0; 0; (Kt*Kta)/(tau*taua);];
C=[1 0 0];



%% cntinuous-time poles 
eigs(A);
%% continuous-time state space
sys_cont = ss(A,B,C,0);

%% Sampling period
 h = 0.002;

%% ZOH based discrete system
sys_disc= c2d(sys_cont, h);
phi = sys_disc.a;
Gamma = sys_disc.b;


%% controllability test
gamma = [Gamma phi*Gamma   phi^2*Gamma ];
det(gamma);


% if det(gamma) == 0
%     disp('Uncontrollable');
% else
%     disp('Controllable');
% end

 
%% discrete-time poles 
eigs(phi);

%% Desired closed-loop poles
 alpha = [0.99 0.99  0.99]; %good results!
%alpha = [0.93 0.96  0.96];

%% feedback gain
K = -acker(phi,Gamma,alpha);
% [G,S,e] = dlqr(phi,Gamma,eye(3),1);
% K = -G;

%% feedforward gain
F = 1/(C*inv(eye(3)-phi-Gamma*K)*Gamma);

position(1)=q0; 
x1(1) = v0;
x2(1) = a0;
x3(1) = da0;
input(1) = 0;
time(2) = h; time(1) = 0;
 
for i=1:16
    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des1;
   %     u = 0;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
        
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;
    
end
for i=17:32
    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des2;
   %     u = 0;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
        
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;
    
end
for i=33:50    
    u = K*[x1(i);x2(i); x3(i)] + F*w_des3;
    xkp1 = phi*[x1(i);x2(i); x3(i)] + Gamma*u;
    x1(i+1) = xkp1(1);
    x2(i+1) = xkp1(2);
    x3(i+1) = xkp1(3);
    input(i+1) = u;    
    time(i+1) = time(i) + h;   
    position(i+1)=position(i)+x1(i)*h+x2(i+1)*h*h/2;    
end
    
qf=position;
vf=x1;
af=x2;
daf=x3;

if abs(max(input))>12
 max(input)
end

end

 function [v,a1]=traj(t0,tf,v0,vf,a0,af,h)

A=[ 1  t0  t0^2    t0^3;
    1  tf  tf^2    tf^3;
    0   1  2*t0  3*t0^2;
    0   1  2*tf  3*tf^2;]
    
 B=[v0;vf;a0;af;]
 
 a=A\B
 
 t=t0:h:tf;

 v=a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3;
 a1=a(2)+2*a(3)*t+3*a(4)*t.^2;

 end