clear all;
clc;
count = 1;
fs = 2;
dt = 1/(fs);

rfo = rfo('COM11', 115200);
rfo.cmd('start -p --nonleaded');

yTarget = [];
y = [];
t = [];
h = plot(0,0);
ylim([0 260]);

while true
    rfo.dev.flush("input");
    temp = str2double(convertStringsToChars(rfo.dev.readline()))
    target = str2double(convertStringsToChars(rfo.dev.readline()))
    pid = str2double(convertStringsToChars(rfo.dev.readline()))

    y(count) = temp;
    yTarget(count) = target;

    t = [t (count-1)*dt];
    set(h,'XData', t, 'YData', y(1:count));
    count = count + 1;

end

rfo.cmd('');
clear rfo;

