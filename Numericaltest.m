/*
*Numerical results Iterative Learning Control for the position control of a DC motor
*@author: Hoang T.Hua
*
/*



clear
clc

A = 0.05903;  B = -0.04243;  C = -0.05058;  D = 0.03506;
E = 1.059;  F = -2.817;  G = 2.498;  H = -0.7395;

%%
N = 78;   % N stand for time
Kd = 0.0993292; Kd1= 0.1; Kd2 = 0.2; Kd3 = 0;Kd4=0.04;
Kp = 0.730924;Kp1=0.2;Kp2=0.5;Kp3=0.6;Kp4=0.8; Kp5 = 1;
r = []; con=1;
t = [];
y = [];
u = [];
e = [];err=[];err2=[];
con = 1;
for i = 1:N
    r(1)=5*(1-cos(0));
    r(i+1) = (1-cos(4*i*0.01))*374/2;
    r(2*N-i+1)=(1-cos(4*i*0.01))*374/2;  % Create the reference
    t(i)= i-1;
    t(N+i)= N+i-1;     % Control time interval       
    e(i)= 0; 
    e(N+i)=0;          % Tracking error                   
end
figure(1)
 subplot 121
 plot(t,r, 'linewidth', 1.5); hold on;
 %plot(t,r, 'k', 'linewidth', 1.5); hold on;
 xlabel('time')
 ylabel('output')
 
subplot 122
xlabel('Iteration [-]')
ylabel('Error norm [m]')

title(' Error norm after each Iteration')
%%
u = r;
%%
trials =50;
for k = 1:trials
    %while con ==1
    for i = 1:2*N-1
        if i==1
        u(i)= u(i)+ Kp*e(i) + Kd*(e(i)-0);  % update control signal
        else
        u(i)= u(i)+ Kp*e(i) + Kd*((e(i)-e(i-1))/0.01);
        end
    end
    u(2*N)= u(2*N) + Kp*e(2*N) + Kd*(e(2*N)-e(2*N-1));  % update control signal for the last iteration
    y(1) = 0; % initial values
    y(2) = 0.1; % initial values
    y(3) = 0.6;
    for i = 4:2*N
        y(i) = ((A/E)*u(i)) + ((B/E)*u(i-1)) + ((C/E)*u(i-2)) + ((D/E)*u(i-3)) - ((F/E)*y(i-1)) - ((G/E)*y(i-2)) - ((H/E)*y(i-3));
    end
    e = r - y;
    err(k) = sqrt(sum((r-y).^2))/374;   % Norm of error
    figure(1)   
    subplot 121 % Plot output
    hold on;
    p = plot(t,y); p.Color = [1, 0, 0, 0.7]; hold off  
    legend('reference', 'output');
    
    subplot 122   % Plot norm of error
     %a = plot(err, 'r*', 'linewidth', 1.5); 
     a = plot(k, err(k), 'r*', 'linewidth', 1.5); 
     xlim([0 trials]); hold on;
     xlabel('Iteration [-]')
     ylabel('Error norm [p]')
     legend('error norm1')
     title(' Error norm after each Iteration')
     %con = input('Press 1 to continue: ')
end
%
figure (2)  % Plot the last iteration
 plot(t*0.01,r,t*0.01,y,'--', 'linewidth', 1.5); legend('reference', 'output-ILC');
 xlabel('Time (s)')
 ylabel('Output')
 
 
 
 
 
