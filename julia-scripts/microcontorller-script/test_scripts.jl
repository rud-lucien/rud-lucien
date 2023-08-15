"""
    parse_data_line(line::String)

Parse a line of space-separated data values. Drops the first value.

# Arguments
- `line`: A string of space-separated data values

# Returns
- An array of parsed float values
"""
function parse_data_line(line::String)
    str_values = split(line)[2:end]  # Split line and drop first value
    return parse.(Float64, str_values)  # Convert to float array
end

"""
    module SerialPortReader

A module for reading data from a serial port.

This module provides the following functions:

- `read_serial_data(portname::String, baudrate::Int; databits::Int=8, parity::Symbol=:none, stopbits::Int=1, flowcontrol::Symbol=:none)::String`: Read and parse data from the serial port specified by `portname` at the given `baudrate`. Prints parsed data values to the console.
- `start_reading()::Nothing`: Start reading data from the serial port in a separate task.
- `stop_reading()::Nothing`: Stop reading data from the serial port.

"""
module SerialPortReader

    # Import serial port package
    using LibSerialPort

    # Define global variables
    const portname = "COM9"  # Arduino serial port
    const baudrate = 9600  # Communication baud rate
    global continue_reading = false

    """
        read_serial_data(portname::String, baudrate::Int; databits::Int=8, parity::Symbol=:none, stopbits::Int=1, flowcontrol::Symbol=:none)::String

    Read and parse data from the serial port specified by `portname` at the given `baudrate`.
    Prints parsed data values to the console.

    # Arguments
    - `portname`: Name of the serial port (String)
    - `baudrate`: Baud rate for the serial communication (Int)
    - `databits`: Number of data bits (Int, default=8)
    - `parity`: Parity type (Symbol, default=:none)
    - `stopbits`: Number of stop bits (Int, default=1)
    - `flowcontrol`: Flow control type (Symbol, default=:none)

    # Returns
    - `data_str`: String containing the read data
    """
    function read_serial_data(portname::String, baudrate::Int; databits::Int=8, parity::Symbol=:none, stopbits::Int=1, flowcontrol::Symbol=:none)::String
        try
            LibSerialPort.open(portname, baudrate, databits, parity, stopbits, flowcontrol) do sp
                while (continue_reading)
                    if bytesavailable(sp) > 0
                        data = readuntil(sp, '\n')
                        data_str = String(data)
                        return data_str
                    end
                    sleep(0.1)  # Pause to prevent high CPU usage
                end
            end
        catch e
            println("Could not open port $portname: $e")  # Error handling
        end
    end

    """
        start_reading()

    Start reading data from the serial port in a separate task.

    # Arguments
    - None

    # Returns
    - None
    """
    function start_reading()::Nothing
        global continue_reading = true
        @async read_serial_data(portname, baudrate)  # Run in a separate task
    end

    """
        stop_reading()

    Stop reading data from the serial port.

    # Arguments
    - None

    # Returns
    - None
    """
    function stop_reading()::Nothing
        global continue_reading = false
    end

    # Export functions
    export read_serial_data, start_reading, stop_reading

end