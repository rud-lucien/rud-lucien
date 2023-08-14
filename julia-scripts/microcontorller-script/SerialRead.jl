#=
Purpose: Read serial data from Arduino and plot it in real time
=#

# Import serial port package
using LibSerialPort

availablePorts = get_port_list()
selectedPort = availablePorts[3]
portname = "COM9"  # Arduino serial port
baudrate = 9600  # Communication baud rate

# Flag to control data reading
global continue_reading = false

"""
    read_serial_data(portname::String, baudrate::Int)

Read and parse data from the serial port specified by `portname` at the given `baudrate`.
Prints parsed data values to the console.

# Arguments
- `portname`: Name of the serial port (String)
- `baudrate`: Baud rate for the serial communication (Int)

# Returns
- None
"""
function read_serial_data(portname::String, baudrate::Int)
    try
        LibSerialPort.open(portname, baudrate) do sp
            while (continue_reading)
                if bytesavailable(sp) > 0
                    data = readuntil(sp, '\n')
                    data_str = String(data)
                    data_values = parse_data_line(data_str)
                    println(data_values)
                end
                sleep(0.1)  # Pause to prevent high CPU usage
            end
        end
    catch e
        println("Could not open port $portname: $e")  # Error handling
    end
end

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
    start_reading()

Start reading data from the serial port in a separate task.

# Arguments
- None

# Returns
- None
"""
function start_reading()
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
function stop_reading()
    global continue_reading = false
end