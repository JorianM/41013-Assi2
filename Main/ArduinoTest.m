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
                disp('E-stop Pressed')
            else
                disp('Robot Resume Motion')
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
