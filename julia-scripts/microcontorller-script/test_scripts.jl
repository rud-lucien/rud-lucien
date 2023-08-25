# Load packages that will be used
using GLMakie
using Observables
using DataStructures: CircularBuffer
include("SerialPortReader.jl")
using .SerialPortReader  # Import your SerialPortReader module
using Dates

# Define an empty DataFrame outside the function
df = DataFrame(Float32[], Symbol[:col1, :col2, :col3, :col4, :col5])

# Define a global variable to control the viewing process
global continue_viewing = true

"""
    parse_data(data::Array{SubString{String},1})

Parse the data read from the serial port. This function needs to be implemented.

# Arguments
- `data`: Data read from the serial port (Array{SubString{String},1})

# Returns
- Parsed data
"""
function parse_data(data::Array{SubString{String},1})
    # Remove any non-numeric strings
    numeric_data = filter(x -> isdigit(x[1]) || x[1] == '.', data)

    # Convert each string in the array to a float
    float_data = map(x -> parse(Float64, x), numeric_data)
    
    # If there are more than five elements, keep only the last five
    if length(float_data) > 5
        float_data = float_data[end-3:end]
    end

    return float_data
end

"""
    view_data(portname::String, baudrate::Int)

Starts reading data from the serial port, parses the data, adds an elapsed time column in seconds, 
and prints the data to the console.

# Arguments
- `portname`: Name of the serial port (String)
- `baudrate`: Baud rate for the serial communication (Int)

# Returns
- `nothing`
"""
function view_data(portname::String, baudrate::Int)
    global continue_viewing = true
    data_ch = SerialPortReader.start_reading(portname, baudrate)
    start_time = now()
    
    # While loop to continuously read data
    @async while continue_viewing
        data = take!(data_ch)
        parsed_data = parse_data(data)
        elapsed_time = (now() - start_time).value / 1000
        parsed_data = [elapsed_time; parsed_data]
        
        # Add parsed_data to the DataFrame
        push!(df, parsed_data')
        
        @show parsed_data
    end
end

"""
    stop_viewing()

Stops the data viewing process.

# Returns
- `nothing`
"""
function stop_viewing()
    global continue_viewing = false
    SerialPortReader.stop_reading()
end

# Call the functions
view_data("COM12", 9600)
sleep(15)
stop_viewing()

fig = Figure(resolution = (800, 600))
ax = Axis(fig[1, 1])

# Create an Observable to hold the mouse position text
mouse_position = Observable("Mouse position: (0, 0)")

# Add the text to the plot
text!(ax, mouse_position, position = (10, 10), fontsize = 20)

# Update the Observable with the current mouse position whenever the mouse is moved
on(fig.scene.events.mouseposition) do pos
    mouse_position[] = "Mouse position: " * string(pos)
end
scatter!(ax, markersize = ms, color = color)
fig