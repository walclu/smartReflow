load('test3.mat');

subplot(211);
plot(y);

Gs = tf(1, [18 1 0]);
subplot(212);
step(Gs,250)
