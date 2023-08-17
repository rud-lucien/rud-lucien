# Import necessary packages
using LibSerialPort
using Dates
using DataFrames

# Global flag to control the data reading process
global continue_reading = false

# Function to open a serial port given a port name and baud rate
function open_serial_port(portname::String, baudrate::Int)::SerialPort
    try
        port = LibSerialPort.open(portname, baudrate)
        println("Serial port opened successfully!")
        return port
    catch e
        println("Could not open port $portname: $e")  # Error handling
    end
end

# Function to close a serial port
function close_serial_port(port::SerialPort)::Nothing
    close(port)
    return nothing
end

# Function to check if a serial port is open or closed
function check_serial_port(port::SerialPort)::Bool
    if isopen(port)
        println("Serial port open")
        return true
    else
        println("Serial port not open")
        return false
    end
end

# Function to read data from a serial port
# It reads data continuously in an asynchronous loop
function read_serial_data(port::SerialPort)::Union{Channel{Array{SubString{String},1}}, Nothing}
    if check_serial_port(port) == true
        data_ch = Channel{Array{SubString{String},1}}(1)

        @async while true
            data = readuntil(port, '\n')
            data_str = String(data)
            str_values = split(data_str)  # Split the string into an array of substrings
            str_values = str_values[2:end]  # Drop the first element
            put!(data_ch, str_values)
            sleep(0.1)  # Pause to prevent high CPU usage
        end

        return data_ch
    else
        println("Serial port not open")
        return nothing
    end
end

# Function to parse data from an array of strings to an array of floats
function parse_data(data::Array{SubString{String},1})
    # Remove any non-numeric strings
    numeric_data = filter(x -> isdigit(x[1]) || x[1] == '.', data)

    # Convert each string in the array to a float
    float_data = map(x -> parse(Float64, x), numeric_data)

    return float_data
end

# Function to stop reading data for live plot
function stop_reading_data_for_live_plot()
    global continue_reading = false
end

# Function to start reading data for live plot
function start_reading_data_for_live_plot(incoming_data::Union{Channel{Array{SubString{String},1}}, Nothing}, df::DataFrame)
    global continue_reading = true

    if incoming_data == nothing
        println("No data channel available")
        return
    end

    start_time = now()
    @async while continue_reading
        data = take!(incoming_data)
        parsed_data = parse_data(data)
        elapsed_time = (now() - start_time).value / 1000
        parsed_data = [elapsed_time; parsed_data]
        push!(df, parsed_data)
        sleep(0.1)  # Pause to prevent high CPU usage
    end
end

# Define a DataFrame to store the live data
df_live = DataFrame(DataFrame(Time=Float64[], Humidity=Float64[], Temperature_C=Float64[], Temperature_F=Float64[], Heat_Index_C=Float64[], Heat_Index_F=Float64[]))

# Open the serial port and start reading data
data_ch = open_serial_port("COM12", 9600)
incoming_data = read_serial_data(data_ch)

# Start reading data for live plot
start_reading_data_for_live_plot(incoming_data, df_live)

# Wait for some data to be read
sleep(5)

# Stop reading data for live plot
stop_reading_data_for_live_plot()

# Close the serial port
close_serial_port(data_ch)

# Display the DataFrame in a table
vscodedisplay(df_live)

# Check if the serial port is closed
check_serial_port(data_ch)