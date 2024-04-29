clear all;
close all;
clc;
% Constants
M1 = 100;
M2 = 100;
K1 = 5;
K3 = 5;
K2 = 50;
F1 = 100; 
F2 = 100;

% Req2
B1 = tf ([M1,0,0],1);
B2 = tf ([F1,0],[1]);
B3 = tf ([K2],1);
B4 = tf (1,[K1]);
B5 = tf ([K2],[1]);
B6 = tf ([K3],[1]);
B7 = tf ([M2,0,0],[1]);
B8 = tf ([F2,0],[1]);
B9 = tf ([K2],[1]);
B10 = tf ([1],[K2]);
B11= tf ([1],[1]); % AN INPUT BLOCK OF GAIN = 1 TO BE USED AS AN INPUT BLOCK
BlockMat = append(B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11);
%BlockMat = append(B1,B4,B2,B3,B5,B6,B7,B9,B8,B10)

connectionMap = [1, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               2, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               4, -1, -3, -2, 5, 11, 0, 0, 0, 0, 0;...
               5, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               6, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               7, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               9, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
               10, -6, -8, -7, 9, 0, 0, 0, 0, 0, 0;...
               ];


input = 11;
output = [4,10];

system = connect(BlockMat, connectionMap, input, output);

clc;
tf1 = tf(system(1));
tf2 = tf(system(2));
% Display the first transfer function
disp('Transfer Function 1:');
disp(tf1);

% Display the second transfer function
disp('Transfer Function 2:');
disp(tf2);


% Req 3 ( x repesents the poles and o represents the zeroes)
figure;

% Plot the poles of the first transfer function on the first subplot
subplot(2, 1, 1);
pzmap(tf1);
title('Poles of the first transfer function');
p1 = findobj(gca, 'Type', 'line'); 
set(p1, 'Color', 'k', 'LineWidth', 2); 
% Plot the poles of the second transfer function on the second subplot
subplot(2, 1, 2);
pzmap(tf2);
title('Poles of the second transfer function');
p2 = findobj(gca, 'Type', 'line'); 
set(p2, 'Color', 'r', 'LineWidth', 2); 


% Req 4
[y1,t1]=step(tf1);
[y2,t2]=step(tf2);

steadyStateValue1 = dcgain(tf1);
disp(steadyStateValue1)
steadyStateValue2 = dcgain(tf2);
disp(steadyStateValue2)


figure;
subplot(2, 1, 1);
plot(t1, y1);
text(mean(t1), steadyStateValue1, ['SS= ', num2str(steadyStateValue1)], 'HorizontalAlignment', 'center');
title('Step Response of Transfer Function 1');
xlabel('Time');
ylabel('X1');
grid on;
subplot(2, 1, 2);
plot(t2, y2);
text(mean(t2), steadyStateValue2, ['SS= ', num2str(steadyStateValue2)], 'HorizontalAlignment', 'center');
title('Step Response of Transfer Function 2');
xlabel('Time');
ylabel('X2');
grid on;

% Req 5,6,7 (convert the second part of the system after block diagram
% reduction to a unity feedback system)

BR1 = tf(0.005, [1 ,2 ,2.1 ,1.1 ,0.0525]);
BR2 = tf ([1],[1]);
BlockMat2 = append(BR1,BR2);
connectionMap2 = [1, 2,-1;];
input2 = 2;
output2 = 1;
new_system_x2 = connect(BlockMat2,connectionMap2,input2 ,output2);
tf3 = tf(new_system_x2);
[y3,t3] = step(2*tf3);  % desired deisplacement is 2
steadyStateValue3 = dcgain(2*tf3);
figure;
subplot(2, 1, 1);
plot(t3, y3);
text(mean(t3), steadyStateValue3, ['SS= ', num2str(steadyStateValue3)], 'HorizontalAlignment', 'center');
title('Step Response of Transfer Function 1');
xlabel('Time');
ylabel('X2');
grid on;
subplot(2, 1, 2);
pzmap(tf3);
title('Poles of the second transfer function');
p3 = findobj(gca, 'Type', 'line'); 
set(p3, 'Color', 'r', 'LineWidth', 2);

info = stepinfo(tf3*2);
disp(info)
RiseTime = info.RiseTime;
PeakTime = info.PeakTime;
SettlingTime = info.SettlingTime;
disp(['Rise Time: ', num2str(RiseTime)]);
disp(['Peak Time: ', num2str(PeakTime)]);
disp(['Maximum Peak: ', num2str(steadyStateValue3)]);
disp(['Settling Time: ', num2str(SettlingTime)]);
ess3=(2-steadyStateValue3);
disp(['ess: ', num2str(ess3)]);


% Req 8
Kp_values = [1, 10, 100, 1000];
rise_times = zeros(size(Kp_values));
peak_times = zeros(size(Kp_values));
max_peaks = zeros(size(Kp_values));
settling_times = zeros(size(Kp_values));
ess_values = zeros(size(Kp_values));
figure;
closed_loop_tfs = [];
for i = 1:length(Kp_values)
    % Proportional controller
    Kp = Kp_values(i);
    B1=tf([Kp],[1]);
    BlockMat = append(B1,BR1,BR2);
    connectionMap = [1, 3,-2;2,1,0;];
    input =3;
    output=2;
    % Simulate the closed-loop system
    new_system_x2 = connect( BlockMat,connectionMap,input ,output);
    closed_loop_tf = tf(new_system_x2);
    closed_loop_tfs = [closed_loop_tfs;closed_loop_tf];
    [y, t] = step(2*closed_loop_tf);
    steadyStateValue4 = dcgain(2*closed_loop_tf);
    subplot(2, 2, i);
    plot(t, y);
    text(mean(t), steadyStateValue4, ['SS= ', num2str(steadyStateValue4)], 'HorizontalAlignment', 'center');
    title(['Step Response with Kp = ', num2str(Kp)]);
    xlabel('Time');
    ylabel('Output');
    grid on;
    % Calculate step response characteristics
    info = stepinfo(2*closed_loop_tf);
    
    % Extract transient response parameters
    rise_times(i) = info.RiseTime;
    peak_times(i) = info.PeakTime;
    max_peaks(i) = info.Peak;
    settling_times(i) = info.SettlingTime;
    
    % Calculate steady-state error
    ess_values(i) = 2 - steadyStateValue4;
end
figure;

for i = 1:length(closed_loop_tfs)
    % Create subplot
    subplot(2, 2, i);
    
    % Plot pole-zero map
    pzmap(closed_loop_tfs(i));
    p = findobj(gca, 'Type', 'line'); 
    set(p, 'Color', 'r', 'LineWidth', 2);
    % Add title
    title(['Transfer Function of kp ', num2str(Kp_values(i))]);
end
% Display the transient response parameters and adjusted steady-state error
disp('Effect of Proportional Controller Gain (Kp) on Transient Response Parameters:');
disp('--------------------------------------------------------------------------');
%disp(' Kp | Rise Time | Peak Time | Max Peak | Settling Time | Adjusted Steady-State Error');
%disp('--------------------------------------------------------------------------');
disp(' Kps: ')
disp(Kp_values)
disp(' Rise Time: ')
disp(rise_times)
disp('Peak Time: ')
disp(peak_times)
disp('Max Peak: ')
disp(max_peaks)
disp(' Settling Time: ')
disp(settling_times)
disp(' ess: ')
disp(ess_values)

% Req 9
Kp = 4189.5; % From hand analysis
B1=tf([Kp],[1]);
BlockMat = append(B1,BR1,BR2);
connectionMap = [1, 3,-2;2,1,0;];
input = 3;
output = 2;
% Simulate the closed-loop system
new_system_x2 = connect(BlockMat,connectionMap,input ,output);
closed_loop_tf = tf(new_system_x2);
% Desired displacement of the second mass is to be 4 m
[y,t] = step(4*closed_loop_tf);
ess = abs(4-y(end));
disp(['ess of proportional-only controller = ', num2str(ess)]);

figure();
plot(t, y);
title('Step Response (proportional-only controller with Kp=4189.5)');

figure();
pzmap(closed_loop_tf);
title('Poles (proportional-only controller with Kp=4189.5)');

% Req 10 (Change Kp and Ki values by trial and error)
Kp = 100;
Ki = 10;
s = tf('s');
PI_block = Kp + Ki/s;
BlockMat = append(PI_block, BR1, BR2);
connectionMap = [1, 3,-2;2,1,0;];
input = 3;
output = 2;
new_system = connect(BlockMat, connectionMap, input, output);
new_system_tf = tf(new_system);

figure();
plt = stepplot(new_system, stepDataOptions('InputOffset',0,'StepAmplitude',4));
setoptions(plt,'RiseTimeLimits',[0,1]);

[y,t] = step(4*new_system);
ess = abs(4 - y(end));

figure();
pzmap(new_system_tf);

info = stepinfo(2*new_system_tf);

disp('Effect of PI (Kp and Ki) on Transient Response Parameters:');
disp('--------------------------------------------------------------------------');
disp(' Rise Time | Peak Time | Max Peak | Settling Time | Steady-State Error');
disp('--------------------------------------------------------------------------');
disp([info.RiseTime, info.PeakTime, info.Peak, info.SettlingTime, ess]);
