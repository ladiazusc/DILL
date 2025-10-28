% Define COM port and baud rate
port = "COM13";
baud = 115200;

%%
% Input Parameters for Testing
HowManyTimes = 3;
CMtoMoveY = 5;
CMtoMoveX = -2;
Pause = 0.0; % 

StepY = 1.3; % Callibrated
StepX = 0.25; % Callibrated
GCode = string(CMtoMoveY*Step);
GxCode = string(CMtoMoveX*StepX);


% Create serialport object
s = serialport(port, baud);

% Wait 1 second for device reset
pause(1.5);

    
    % If you expect a reply, read it
    while( s.NumBytesAvailable > 0)
        response = readline(s);
        disp("Response: " + response);
    end
    for i = 1:HowManyTimes
        % Example command to send
        cmd = "G0 X" + GxCode + " Y" + GCode; % Step Size

        % Send command
        writeline(s, cmd);


        % Wait 1 second before sending again
        pause(Pause); % rate
    end


% Cleanup (this will only run if you break the loop with Ctrl-C)
clear s;
