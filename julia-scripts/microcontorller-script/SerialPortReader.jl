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
    start_reading(portname::String, baudrate::Int; print_data_to_console::Bool=false)

    Starts reading data from the serial port specified by `portname` at the given `baudrate`. 
    This function starts two asynchronous tasks: one for reading the data from the serial port 
    and another for processing the data lines. If `print_data_to_console` is set to true, the 
    read data is printed to the console.

    # Arguments
    - `portname`: Name of the serial port (String)
    - `baudrate`: Baud rate for the serial communication (Int)
    - `print_data_to_console`: If set to true, the read data is printed to the console. (Bool, default=false)

    # Returns
    - A Channel that the read data is pushed into
    """
    function start_reading(portname::String, baudrate::Int; print_data_to_console::Bool=false)
        global continue_reading = true
        ch = Channel{Array{SubString{String},1}}(Inf)  # Create a channel to store the data lines
        data_ch = Channel{Array{SubString{String},1}}(Inf)  # Create a channel to return the data
        @async read_serial_data(ch, portname, baudrate)  # Run read_serial_data in a separate task
        @async while continue_reading  # Process the data lines in another task
            data_line = take!(ch)  # Take a data line from the channel
            if print_data_to_console
                println(data_line)  # print the data line to the console
            end
            put!(data_ch, data_line)  # put the data line into the data channel
        end
        return data_ch  # return the data channel
    end

    """
        stop_reading()

    Stops reading data from the serial port.

    # Returns
    - `nothing`
    """
    function stop_reading(port::SerialPort)::Nothing
        close(port)
        global continue_reading = false
        return nothing
    end

    # Export the functions
    export start_reading, stop_reading

end