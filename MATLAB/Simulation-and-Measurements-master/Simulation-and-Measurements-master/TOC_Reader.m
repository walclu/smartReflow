%% Read temperature curve from file
file = '2018-05-04-12:39-PDreg.txt';
data = importdata(file);
expression = '<temp><Ts>%f</Ts><T>%f</T><t>%f</t></temp>';

N = length(data);
Ts = nan(N,1);
T = nan(N,1);
t = nan(N,1);
for i = 1:N
    values = sscanf(data{i}, expression);
    Ts(i) = values(1);
    T(i) = values(2);
    t(i) = values(3);
end

plot(t, Ts)
hold on
plot(t, T)
hold off
grid on
xlabel('t / s')
ylabel('T / °C')
legend('T_{set}', 'T')

%% Live temperature curve from COM port
port = '/dev/ttyACM0';
expression = '<temp><Ts>%f</Ts><T>%f</T><t>%f</t></temp>';

Ts = [];
T = [];
t = [];
s = serial(port);
fopen(s); 

%%
fig = figure;

while 1
    try
        str = fgetl(s)
        values = sscanf(str, expression);
        Ts = [Ts values(1)];
        T  = [T  values(2)];
        t  = [t  values(3)];
        
        plot(t, Ts)
        hold on
        plot(t, T)
        hold off
        grid on
        xlabel('t / s')
        ylabel('T / °C')
        legend('T_{set}', 'T','location','northwest')
    catch

    end
    
    drawnow
    pause(0.2)
    
    % close window
    if ~ishghandle(fig)
        break
    end
end
%%
fclose(s);