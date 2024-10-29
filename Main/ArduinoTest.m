classdef ArduinoTest < handle
   
    properties
        SerialObj % Serial port object

    end
    
    methods
        function obj = ArduinoTest(port, baudRate)
            % Constructor: Initialize and configure the serial port connection
            
            % Step 1: Close any existing serialport connections
            existingPorts = serialportlist("all");
            for i = 1:length(existingPorts)
                try
                    delete(serialport(existingPorts(i)));
                catch ME
                    warning('Failed to delete serial port %s: %s', existingPorts(i), ME.message);
                end
            end
            
            % Step 2: Connect to the Arduino
            obj.SerialObj = serialport(port, baudRate);
            
            % Step 3: Configure the serial port object
            configureTerminator(obj.SerialObj, "CR/LF");
            flush(obj.SerialObj);
            
            % Step 4: Initialize UserData
            obj.SerialObj.UserData = struct("Data", [], "LastValue", NaN); % Keep track of last value
            
            % Step 5: Set up the callback function for reading data
            configureCallback(obj.SerialObj, "terminator", ...
                              @(src, event) obj.readButtonData(src, event));
        end
        
        function readButtonData(obj, src, ~)
            % Callback function to read data from the serial port
          %  global stopFlag;
            % Read the ASCII data
            global currentValue;
            data = readline(src);
            currentValue = str2double(data); % Convert the received data to numeric
            
            % Store the value in UserData
            src.UserData.Data(end+1) = currentValue;
            
            % Check for value change
            if currentValue ~= src.UserData.LastValue
                src.UserData.LastValue = currentValue; % Update last value
                fprintf('Button State Changed: %d\n', currentValue);
            end
            
            if currentValue ==1
             %   stopFlag = true;  % Set the stop flag to true
                disp('stopflag true')
            else
             %   stopFlag = false;
            end
            % Print the current value
            fprintf('Current Value: %d\n', currentValue);
        end

        
    

        function delete(obj)
            % Destructor: Clean up the serial port object
            if isvalid(obj.SerialObj)
                configureCallback(obj.SerialObj, "off"); % Turn off callbacks
                delete(obj.SerialObj); % Delete the serial port object
            end
        end
    end
end

    %{
    properties
        Arduino % Arduino object
        ButtonPin % Digital pin for the pushbutton
        LedPin % Digital pin for the LED
        StopFlag % Flag to indicate if E-stop has been activated
        Timer
    end

    methods
        function obj = ArduinoTest(comPort, boardType, buttonPin, ledPin)
            % Constructor for EStop class
            obj.Arduino = arduino(comPort, boardType);
            obj.ButtonPin = buttonPin;
            obj.LedPin = ledPin;
            
            % Configure the Arduino pins
            configurePin(obj.Arduino, obj.LedPin, 'DigitalOutput');
            configurePin(obj.Arduino, obj.ButtonPin, 'DigitalInput');
            obj.StopFlag = false;

            % Set up a timer for monitoring the E-stop button
            obj.Timer = timer('ExecutionMode', 'fixedRate', ...
                              'Period', 0.1, ... % Check every 100 ms
                              'TimerFcn', @(~,~) obj.checkButton());
            start(obj.Timer); % Start the timer immediately in the constructor
        end
        
        function startMonitoring(obj)
            % Start the timer to monitor the button
            if ~strcmp(obj.Timer.Running, 'on')
                start(obj.Timer);
            end
        end
        
        function stopMonitoring(obj)
            % Stop the timer
            if strcmp(obj.Timer.Running, 'on')
                stop(obj.Timer);
                delete(obj.Timer);
                obj.Timer = timer('ExecutionMode', 'fixedRate', ...
                                  'Period', 0.1, ... % Check every 100 ms
                                  'TimerFcn', @(~,~) obj.checkButton()); % Reinitialize the timer
            end
        end
        
        function checkButton(obj)
            % Check the button state
            buttonState = readDigitalPin(obj.Arduino, obj.ButtonPin);
            if buttonState == 1
                obj.StopFlag = true; % Activate the stop flag
                writeDigitalPin(obj.Arduino, obj.LedPin, 0); % Turn on LED
                disp('Emergency stop activated.');
                obj.stopMonitoring(); % Stop monitoring after activation
            else
                writeDigitalPin(obj.Arduino, obj.LedPin, 1); % Turn off LED
            end
        end
        
        function stop = isStopped(obj)
            % Method to check if E-stop has been activated
            stop = obj.StopFlag;
        end
        
        function delete(obj)
            % Cleanup method to delete Arduino object
            stopMonitoring(obj); % Ensure timer is stopped
            clear obj.Arduino;
        end
    end
    %}