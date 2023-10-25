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
function read_serial_data(
    port::SerialPort,
)::Union{Channel{Array{SubString{String},1}},Nothing}
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
function start_reading_data_for_live_plot(
    incoming_data::Union{Channel{Array{SubString{String},1}},Nothing},
    df::DataFrame,
)::Nothing
    global continue_reading = true

    # Re-initialize df_live
    empty!(df)


    if incoming_data == nothing
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

################################################################################
# DataFrame
################################################################################

# Define a DataFrame to store the live data
df_live = DataFrame(
    Time = Float64[0.0],
    Humidity = Float64[0.0],
    Temperature_C = Float64[0.0],
    Temperature_F = Float64[0.0],
    Heat_Index_C = Float64[0.0],
    Heat_Index_F = Float64[0.0],
)

# Define Observables for the live data
obs_time = Observable(df_live.Time)
obs_humidity = Observable(df_live.Humidity)

################################################################################
# Initialize the serial port
################################################################################

fig = Figure(resolution = (800, 600))


ports = get_port_list()
baudrates = [9600, 19200, 38400, 57600, 115200]

portsmenu = Menu(fig, options = ports, fontsize = 20)
baudratemenu = Menu(fig, options = baudrates, fontsize = 20)

data_ch = Observable{Union{SerialPort,Nothing}}(nothing)


open_button = Button(fig, label = "Open Port", height = 60, width = 250, fontsize = 20)
close_button = Button(fig, label = "Close Port", height = 60, width = 250, fontsize = 20)

led_color = [:honeydew4, :chartreuse]
assigned_led_color = Observable(led_color[1])
led = Box(
    fig,
    color = assigned_led_color,
    height = 30,
    width = 30,
    strokewidth = 3,
    strokecolor = :grey63,
)

fig[1, 1] = vgrid!(
    Label(fig, "Ports:", fontsize = 30, width = 400),
    portsmenu,
    Label(fig, "Baudrate:", fontsize = 30, width = 400),
    baudratemenu,
    Label(fig, "", fontsize = 30, width = 400),
    open_button,
    led,
    close_button;
    tellheight = false,
    width = 500,
)


selected_port = Observable(ports[1])
selected_baudrate = Observable(baudrates[1])

on(portsmenu.selection) do select
    selected_port[] = select
    println("Selected port: $select")
end

on(baudratemenu.selection) do select
    selected_baudrate[] = select
    println("Selected baudrate: $select")
end

on(open_button.clicks) do _
    # Open the serial port and start reading data
    data_ch[] = open_serial_port(selected_port[], selected_baudrate[])
    assigned_led_color[] = led_color[2]
    println("Port opened")
end

on(close_button.clicks) do _
    # Close the serial port
    if data_ch[] != nothing
        close_serial_port(data_ch[])
        assigned_led_color[] = led_color[1]
        println("Port closed")
    else
        println("No port to close")
    end
end

################################################################################
# Live Plot
################################################################################

# Define a figure to plot the live data
ax1 = Axis(fig[1, 2])
limits!(ax1, 0, 300, 0, 100)




################################################################################
# Live Plot and Buttons
################################################################################
# Define a figure to plot the live data
ax1 = Axis(fig[1, 2])
fig[2, 2] = buttongrid = GridLayout(tellwidth = false)
labels = ["Start", "Stop", "Clear"]

buttons =
    buttongrid[1, 1:3] =
        [Button(fig, label = l, height = 60, width = 250, fontsize = 20) for l in labels]

on(buttons[1].clicks) do _
    incoming_data = read_serial_data(data_ch[])
    start_reading_data_for_live_plot(incoming_data, df_live)
    limits!(ax1, 0, 300, 0, 100)
    lines!(
        ax1,
        obs_time,
        obs_humidity,
        color = :red,
        linewidth = 2,
        linestyle = :solid,
        label = "Humidity",
    )
    println("Start Reading Data")
end

on(buttons[2].clicks) do _
    stop_reading_data_for_live_plot()
    println("Stop Reading Data")
end

on(buttons[3].clicks) do _
    empty!(ax1)
    println("Cleared Plot")
end

display(fig)
