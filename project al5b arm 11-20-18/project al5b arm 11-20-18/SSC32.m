classdef SSC32 < handle
    %SSC32 is the servo controller that is used to control multiple
    %servos at a same time. In MABL, we use this to control hexapod and
    %arm robot. It can be used to control any robot and application that
    %relates to servos.
    %
    %Authors:  Vu Nguyen        (vkn3505@rit.edu)
    %          Ryan M. Bowen    (rmb3518@rit.edu)
    %
    %Modified: 10/5/2011 (rmb3518)
    
    
    properties(Constant)
        
        minValue = 500;     % Absolute Minimum Servo Value
        maxValue = 2500;     % Absolute Maximum Servo Value
        
    end
    
    properties(SetAccess=private, GetAccess=private)
        
        delayTime   = 0;        %   Delay time for servo movement
        portName    = 'COM1';   %   Communication Port default 'COM1'
        connected   = 0;        %   Flag for connection status
        sPort;                  %   Serial Port Object for communication
        
    end
    
    
    methods(Access=public)
        
        function obj = SSC32(portName)
            % SSC32 constructor
            %
            % @params   portName - communication port name ie 'COM1'
            
            obj.portName = portName;
            obj.sPort = SerialPort(portName);
        end
        
        
        function delete(obj)
            % SSC32 Destructor - disconnect from serial port and destroy
            % objects accorrdingly.
            
            obj.disconnect();
            obj.sPort.delete();
            clear obj;
            display('SSC-32 destroyed!');
        end
        
        function success = setServoValues(obj,valueIn,channelIn)
            % Sets the values of the specified servos.
            % length(valueIn) must equal length(channelIn)
            %
            % @param    valueIn   - servo values
            %           channelIn - respectful channels for servos values.
            %
            % @returns  '1' if success, otherwise error is thrown.
            
            channelLength = length(channelIn) ;
            valueLength = length(valueIn) ;
            
            success = 1;
            
            % Throw an error if not the same lengths
            if (channelLength ~= valueLength)
                error ('"servoChannels" and "servoValues" are not equal in length!!!') ;
            end
            
            % Create the command to send to the servos
            command = obj.makeCommand(valueIn, channelIn);
            
            % Write out the command to the serial port.
            obj.sPort.write(command);
        end
        
        function obj = connect(obj)
            % Connects to the SSC-32 controller using serial port.
            
            display('Attempting to connect to SSC-32 board...');
            obj.sPort.open();
            obj.connected = 1;
            display('Connected to SSC-32!');
        end
        
        function obj = disconnect(obj)
            % Disconnects from SSC-32 by closing serial port.
            
            if( obj.connected)
                obj.sPort.close();
                obj.connected = 0;
                display('SSC-32 disconnected!');
            end
        end
        
    end
    
    methods(Access=private)
        
        function command = makeCommand(obj,valueIn,channelIn)
            % Create the string command to set servos accordingly
            %
            % @param    valueIn   - servo values
            %           channelIn - respectful channels for servos values.
            %
            % @returns  command - the string command.
            
            %Insure that decimal numbers are used.
            valueIn = round(valueIn);
            
            % Initialize the command array
            command = [];
            
            % Build the command string
            for i = 1 : length(channelIn)
                % Truncate bounds if need be
                servoValue = obj.setBound(valueIn(i));
                % Concat command
                command = [command, sprintf('#%iP%iT%i ', channelIn(i), servoValue, obj.delayTime)] ;
            end
            
            % Terminate the command string
            command = [command, sprintf('\n')] ;
            
        end
        
        function servoValue = setBound(obj,servoValue)
            % Truncates a servo value to its absolute bounds
            %
            % @params servoValue - servo value to be bounded
            %
            % @returns properly bounded servo value
            
            % Truncate min
            if servoValue < obj.minValue
                servoValue = obj.minValue;
            end
            
            % Truncate max
            if servoValue > obj.maxValue
                servoValue = obj.maxValue;
            end
        end
    end
    
end

