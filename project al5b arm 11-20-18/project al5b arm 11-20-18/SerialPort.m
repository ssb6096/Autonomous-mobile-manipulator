classdef SerialPort < handle
    %SerialPort is a facade created to facilitate writing to the serial
    %port. Default communciation is established on "COM1 9600 8N1 CR"
    %
    %Authors:  Vu Nguyen        (vkn3505@rit.edu)
    %          Ryan M. Bowen    (rmb3518@rit.edu)
    %
    %Modified: 10/5/2011 (rmb3518)
    
    
    properties(SetAccess=private, GetAccess=private)
        port        = [];       % Native serial port object
        portName    = 'COM1';   % COM port name, default='COM1'
        baudrate    = 9600;     % Baudrate, default = 9600
        databits    = 8;        % Number of databits, default = 8
        stopbits    = 1;        % Number of stopbits, defrault = 1
        flowcontrol = 'none';   % Flow control, default = 'none'
        parity      = 'none';   % Parity, default = 'none'
        terminator  = 'CR';     % Terminator, default = 'CR'
        
        connected   = 0;        % Connectivity flag
    end
    
    methods(Access=public)
                
        function obj = SerialPort(portName)
            % SerialPort constructor
            obj.createPort(portName);
            obj.setSerialPort(obj.baudrate,obj.databits,obj.stopbits,...
                obj.flowcontrol, obj.parity,obj.terminator);
        end
        
        function obj = createPort(obj,portName)
            % Creates a new native serial port object 
            obj.port = serial(portName);
        end
        
        function obj = setSerialPort(obj,baudrate,databits,stopbits,flowcontrol,parity,terminator)
            % Sets the serial port settings
            %
            % @params self descriptive
            obj.port.baudrate      = baudrate;
            obj.port.databits      = databits;
            obj.port.stopbits      = stopbits;
            obj.port.flowcontrol   = flowcontrol;
            obj.port.parity        = parity;
            obj.port.terminator    = terminator;
        
        end
        
        function obj = open(obj)
            % Opens the serial port based on the port and settings. Will
            % attempt to close open ports and delete all open port objects.
            
            display('Attempting to connect to Serial Port...');
            
            if(isPortOpen(obj))
                display('Closing old port...');
                obj.close();
            end
            
            % Close out open serial connections
            delete(instrfind('Status','open'));            
        
            % Open the port
            fopen(obj.port);
            
            % Flag for connectivity
            obj.connected = 1;
            
            display('Connected to Serial Port!');            
        end
        
        function obj = close(obj)
            % Attempts to closes the serial port
            if( obj.connected )
                fclose(obj.port);
                obj.connected = 0;
                display('Serial Port closed!');
            end
        end

        function isOpen = isPortOpen(obj)
            % Returns the open state of the serial port
            
            if(strcmp(obj.port.Status,'open'))
                isOpen = true;
            else
                isOpen = false;
            end
        end
        
        function obj = setCOMname(obj,nameCOM)
            % Sets the com port name
            %
            % @params  nameCOM - serial port com name.
            obj.port.Port = nameCOM;
            obj.portName = nameCOM;
        end
        
        function obj = write(obj,data)
            % Writes out data to the serial port. Throws error if the port
            % has been closed or never opened.
            %
            % @params data - data to be sent out on the port.
            
            if (isPortOpen(obj))
                fprintf(obj.port,data);
            else
                error('Need to open serial port');
            end
            
        end
        
        function delete(obj)
            % SerialPort destructor, closes port and destroys objects.
            obj.close();
            obj.portDelete();
            clear obj;
            
            display('Serial Port destroyed!');
        end
        
        function obj = portDelete(obj)
            % Finds and deletes the port object
            delete(instrfind(obj.port));
        end
        
    end
    
end

