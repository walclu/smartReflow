classdef rfo < handle
    properties
        COM;
        baud;
        dev;
    end
    methods
        function obj = rfo(comport, baudrate)
            obj.COM = comport;
            obj.baud = baudrate;
            obj.dev = serialport(obj.COM, obj.baud);
        end

        function cmd(obj, str)
            for i = 1:length(str)
                write(obj.dev, str(i), 'char');
            end
            write(obj.dev, 13, "int8");
        end

    end
end
