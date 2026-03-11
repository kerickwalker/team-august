%% Line follow plot
close all
clear
%% load
%  1    time 0.0000 sec, from Bode (122)
%  2  3 Motor voltage (V) left, right: 0.00 0.00
%  4  5 Wheel velocity (m/s) left, right: -0.0000 0.0000
%  6    Turnrate (rad/s): 0.0000
%  7  8  9 10 Pose x,y,h,tilt (m,m,rad,rad): 0.0000 0.0000 0.0000 3.1256
% 11 .. 29 Edge sensor: left pos, right pos, line valid, crossing, values d d d d d d d d,  white, used, LED high, average white, line detect count, crossing detect count
% 30    Battery voltage (12.02 V)
dd_10 = load('aaa_bode_10.txt'); % Kp = 0.8 % with lead 0.5 - 0.1
dd_11 = load('aaa_bode_11.txt'); % Kp = 0.8 % with lead 1.0 - 0.15
dd_12 = load('aaa_bode_12.txt'); % Kp = 0.8 % with lead 1.0 - 0.15
dd_13 = load('aaa_bode_13.txt'); % Kp = 0.8 % with lead 1.0 - 0.15 - step
dd_14 = load('aaa_bode_14.txt'); % Kp = 0.8 % with lead 1.0 - 0.15 - curved path
dd_15 = load('aaa_bode_15.txt'); % Kp = 0.8 % with lead 1.0 - 0.15 - step
dd_16 = load('aaa_bode_16.txt'); % Kp = 0.5 % with lead 0.8 - 0.15 - step
%%
dd = dd_16;
fig = 100000 + 100 * 16;
%% control
h = figure(fig);
hold off
plot(dd(:,1), dd(:,2))
hold on
plot(dd(:,1), dd(:,3))
% line
plot(dd(:,1), dd(:,13))
plot(dd(:,1), dd(:,11))
plot(dd(:,1), dd(:,6))
legend('left V', 'right V', 'line valid','line pos','turnrate')
xlabel('sec');
grid on
saveas(h,"bode_set_16_line_follow.png")
%% detection
n = 720;
h = figure(fig + 1);
hold off
nn = 1:8
plot(nn, dd(n,15:22),'r')
hold on
for (j = n:n+10)
    plot(nn, dd(j,15:22),'r')
    x = dd(j, 11) + 4;
    plot([x,x], [700, 1000],'r')
end
n = 124
for (j = n:n+10)
    plot(nn, dd(j,15:22),'b')
    x = dd(j, 11) + 4;
    plot([x,x], [700, 1000],'b')
end
grid on
xlabel('sensor number')
ylabel('Normalized sensor value')
title('Two positions (124,720), notice varying floor level.')
saveas(h,"bode_set_a6_720_124.png")