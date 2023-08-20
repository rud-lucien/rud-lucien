# Import necessary packages
using DataFrames
using Dates
using GLMakie
using LibSerialPort
#using Observables

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
        return nothing
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
function parse_data(data::Array{SubString{String},1})::Array{Float64,1}
    # Remove any non-numeric strings
    numeric_data = filter(x -> isdigit(x[1]) || x[1] == '.', data)

    # Convert each string in the array to a float
    float_data = map(x -> parse(Float64, x), numeric_data)

    return float_data
end

# Function to stop reading data for live plot
function stop_reading_data_for_live_plot()::Nothing
    global continue_reading = false
    return nothing
end

# Function to start reading data for live plot
function start_reading_data_for_live_plot(incoming_data::Union{Channel{Array{SubString{String},1}},Nothing}, df::DataFrame)::Nothing
    global continue_reading = true

    # Re-initialize df_live
    empty!(df)

    if incoming_data == empty
        println("No data channel available")
        return nothing
    end

    start_time = now()
    @async while continue_reading
        data = take!(incoming_data)
        parsed_data = parse_data(data)
        elapsed_time = (now() - start_time).value / 1000
        parsed_data = [elapsed_time; parsed_data]
        push!(df, parsed_data)
        notify(obs_time)
        sleep(0.1)
        # Pause to prevent high CPU usage
    end
    return nothing
end

# Define an Observable DataFrame to store the live data
df_live = DataFrame(Time=Float64[0.0], Humidity=Float64[0.0], Temperature_C=Float64[0.0], 
                    Temperature_F=Float64[0.0], Heat_Index_C=Float64[0.0], Heat_Index_F=Float64[0.0])


obs_time = Observable(df_live.Time)
obs_humidity = Observable(df_live.Humidity)


# Define a figure to plot the live data
fig = Figure(resolution = (800, 600))
ax = Axis(fig[1, 1])
limits!(ax, 0, 300, 0, 100)
#on(obs_time) do _
    #reset_limits!(ax)
#end

lines!(ax, obs_time, obs_humidity, color = :red, linewidth = 2, linestyle = :solid, label = "Humidity")


display(fig)

fig[2, 1] = buttongrid = GridLayout(tellwidth = false, height = 100)
labels = ["Start", "Stop", "Clear"]

buttons = buttongrid[1, 1:3] = [
    Button(fig, label = l, height =  60, width = 250, fontsize = 20
    )
    for l in labels
]

on(buttons[3].clicks) do _
    for col in names(df_live)
        df_live[!, col] .= 0
    end
    notify(obs_time)
    lines!(ax, obs_time, obs_humidity, color = :red, linewidth = 2, linestyle = :solid, label = "Humidity")
end


# Open the serial port and start reading data
data_ch = open_serial_port("COM18", 9600)
incoming_data = read_serial_data(data_ch)

# Start reading data for live plot
start_reading_data_for_live_plot(incoming_data, df_live)

sleep(15)

# Stop reading data for live plot
stop_reading_data_for_live_plot()

# Close the serial port
close_serial_port(data_ch)

# Display the DataFrame in a table
vscodedisplay(df_live)

# Check if the serial port is closed
check_serial_port(data_ch)