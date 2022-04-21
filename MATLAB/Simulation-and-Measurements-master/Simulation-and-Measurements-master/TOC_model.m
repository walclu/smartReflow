%% load measurement data and model
load TOC_measurement

Gs = tf(2, [25 1 0]); % IT1 model, values from evaluation
% Step response model vs reference
figure(1)
[y_step,t_step] = step(Gs, 120);
plot(t_step, y_step)
hold on
plot(temperature(20:139) - temperature(1))
grid on
hold off
xlabel('t / s')
ylabel('T / °C')
legend('model','measurement')

%% Bode plot control (B = 1rad/s)
figure(2)
bode(Gs)
hold on
grid on

%Gr = 5*tf([3.3 1], [1/3.3 1]);
Gr = 5*tf([10 1], 1);
%bode(Gr)
margin(Gr*Gs)
hold off

%% Step response control
figure(3)
step(feedback(Gr*Gs, 1))
grid on

%% Generate solder profile (Chip Quick TS391AX50) as time series object
Tp = [25 100 150 183 230 183];
tp = [80  90  30  60  30];
T = [];
for i = 1:length(Tp)-1
    dT = (Tp(i + 1)-Tp(i))/tp(i);
    T = [T Tp(i):dT:Tp(i+1)-dT]; 
end
t = 0:length(T)-1;

figure(4)
plot(t, T)
grid on
title('Lead Solder Profile')
xlabel('t / s')
ylabel('T / °C')
xlim([0 t(end)])

timeObject = timeseries(T,t);
save 'TOC_solder_profile.mat' -v7.3 timeObject