baud = 115200;

availablePorts = serialportlist("all");
disp("Scanning COM ports for Arduino...");

for i = 1:length(availablePorts)
    port = availablePorts(i);
    fprintf("Trying %s...\n", port);
    try
        s = serialport(port, baud);
        pause(2); % Give time for Arduino to reset
        
        % Check if Arduino replies
        if s.NumBytesAvailable > 0
            response = readline(s);
            fprintf("Response from %s: %s\n", port, response);
        else
            fprintf("No immediate response on %s\n", port);
        end
        
        clear s;
    catch ME
        fprintf("Failed to open %s: %s\n", port, ME.message);
    end
end
