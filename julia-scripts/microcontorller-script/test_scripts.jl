# Load packages that will be used
using GLMakie
using Observables
using DataStructures: CircularBuffer
using .SerialPortReader  # Import your SerialPortReader module

# Create a counter variable
counter_increment = 0
counter_reset = 0

"""
    parse_increment(line::Array{SubString{String},1})

Parse a line of data values. Only the last four values are parsed. Increments the timestamp counter.

# Arguments
- `line`: An array of substrings

# Returns
- An array of parsed float values with timestamp as the first element
"""
function parse_increment(line::Array{SubString{String},1})
    global counter_increment
    counter_increment += 1  # Increment the counter
    parsed_values_increment = parse.(Float64, line[end-3:end])  # Convert to float array
    result_increment = [counter_increment; parsed_values_increment]  # Create result array with counter as the first element
    println("Result Increment: ", result_increment)  # Print the result
    return result_increment
end

"""
    parse_reset(line::Array{SubString{String},1})

Parse a line of data values. Only the last four values are parsed. Resets the timestamp counter.

# Arguments
- `line`: An array of substrings

# Returns
- An array of parsed float values with timestamp as the first element
"""
function parse_reset(line::Array{SubString{String},1})
    global counter_reset
    counter_reset = 1  # Reset the counter
    parsed_values_reset = parse.(Float64, line[end-3:end])  # Convert to float array
    result_reset = [counter_reset; parsed_values_reset]  # Create result array with counter as the first element
    println("Result Reset: ", result_reset)  # Print the result
    return result_reset
end

# Create a circular buffer for the parse_reset function
buffer_size_reset = 500  # Define the size of the buffer
buffer_reset = CircularBuffer{Array{Float64,1}}(buffer_size_reset)

# Function to add data from parse_reset function to the buffer and update the observable
function add_data_to_reset_buffer(data)
    push!(buffer_reset, data)
    data_reset_observable[] = collect(buffer_reset)
end

# Define a new parsing function that calls parse_reset and add_data_to_reset_buffer
function parse_and_add_to_buffer(line::Array{SubString{String},1})
    data_reset = parse_reset(line)
    add_data_to_reset_buffer(data_reset)
end

# Create an observable for the data from parse_reset function
data_reset_observable = Observable(collect(buffer_reset))

# Create a plot that updates whenever the data from parse_reset function changes
fig_reset = Figure()
ax_reset = Axis(fig_reset[1, 1], xlabel = "Time (sec)", ylabel = "Temperature (°C)")
line_reset = lines!(ax_reset, lift(d -> first.(d), data_reset_observable), lift(d -> d[:, 2], data_reset_observable), visible = lift(x -> length(x) > 0, data_reset_observable))

display(fig_reset)

# Start reading data from the serial port
SerialPortReader.start_reading("COM9", 9600, parse_and_add_to_buffer)