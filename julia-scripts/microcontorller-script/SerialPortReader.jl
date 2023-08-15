"""
    SerialPortReader

This module provides functionality to read data from a serial port, 
parse it and pass it to a user-specified function for further processing.
"""
module SerialPortReader

    # Import serial port package
    using LibSerialPort

    # Flag to control data reading
    global continue_reading = false

    """
        read_serial_data(ch::Channel, portname::String, baudrate::Int)

    Reads data from the serial port specified by `portname` at the given `baudrate`, 
    splits the incoming string into substrings, drops the first element and puts the 
    rest into the passed `Channel` `ch`.

    # Arguments
    - `ch`: Channel where the read data is put (Channel)
    - `portname`: Name of the serial port (String)
    - `baudrate`: Baud rate for the serial communication (Int)

    # Returns
    - `nothing`
    """
    function read_serial_data(ch::Channel, portname::String, baudrate::Int)
        try
            LibSerialPort.open(portname, baudrate) do sp
                while (continue_reading)
                    if bytesavailable(sp) > 0
                        data = readuntil(sp, '\n')
                        data_str = String(data)
                        str_values = split(data_str)  # Split the string into an array of substrings
                        str_values = str_values[2:end]  # Drop the first element
                        put!(ch, str_values)  # Put the data line into the channel
                    end
                    sleep(0.1)  # Pause to prevent high CPU usage
                end
            end
        catch e
            println("Could not open port $portname: $e")  # Error handling
        end
    end

    """
        start_reading(portname::String, baudrate::Int, parse_func=println)

    Starts reading data from the serial port specified by `portname` at the given `baudrate`. 
    The data is parsed according to `parse_func`, which should be a function that takes an array of 
    substrings as input and processes it in some way. If `parse_func` is set to `nothing`, the data 
    is read but not processed.

    # Arguments
    - `portname`: Name of the serial port (String)
    - `baudrate`: Baud rate for the serial communication (Int)
    - `parse_func`: Function to parse the read data. Should take an array of substrings as input 
      and process it in some way. If set to `nothing`, the data is read but not processed. 
      (Function, default=println)

    # Returns
    - `nothing`
    """
    function start_reading(portname::String, baudrate::Int, parse_func=println)
        global continue_reading = true
        ch = Channel{Array{SubString{String},1}}(Inf)  # Create a channel to store the data lines
        @async read_serial_data(ch, portname, baudrate)  # Run read_serial_data in a separate task
        @async while continue_reading  # Process the data lines in another task
            data_line = take!(ch)  # Take a data line from the channel
            if parse_func !== nothing
                parse_func(data_line)  # Call the parsing function with the data line
            end
        end
    end

    """
        stop_reading()

    Stops reading data from the serial port.

    # Returns
    - `nothing`
    """
    function stop_reading()
        global continue_reading = false
    end

    # Export the functions
    export start_reading, stop_reading

end