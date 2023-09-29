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

# Function to generate all unique combinations of n colors
function generate_unique_combinations(colors, n)
    # Generate all combinations of n colors
    combinations = vec(join.(Iterators.product([colors for _ = 1:n]...)))

    # Define a helper function to sort characters in a string
    sorted_strings(s) = join(sort(collect(s)))

    # Sort each combination
    sorted_combinations = map(sorted_strings, combinations)

    # Remove duplicates and sort in reverse order
    unique_combinations = sort(unique(sorted_combinations), rev = true)

    return unique_combinations
end

# Function to count the number of times a character appears in each combination
function count_character(character, combinations)
    # Preallocate an array to hold the counts
    ways_to_produce = zeros(Int64, length(combinations))

    # Count the occurrences of the character in each combination
    for i in eachindex(combinations)
        ways_to_produce[i] =
            sum(ifelse.([char == character for char in combinations[i]], 1, 0))
    end

    return ways_to_produce
end

# Function to count occurrences of each character in the combinations
function get_counted_characters(characters, unique_combinations)
    # Count the number of times each character appears in each combination
    counts = [count_character(char, unique_combinations) for char in characters]

    # Convert the array of arrays to a matrix
    counts_matrix = hcat(counts...)

    return counts_matrix
end

# Define colors and number of combinations
colors = ["B", "W"]

n = 2  # Number of combinations

# Characters to count
characters = ['B']

# Generate all unique combinations of n colors
unique_combinations = generate_unique_combinations(colors, n)

# Get counted characters matrix
counts = get_counted_characters(characters, unique_combinations)

# Calculate the product of the counts along the columns and convert the result to a vector
ways = vec(prod(counts, dims = 2))



#=
prototyping code for count_character function

character = 'B'
# Count the number of times the character appears in each combination
ways = Int64[]
character
for i in eachindex(unique_combinations)
    push!(ways, sum(ifelse.([char == character for char in unique_combinations[i]], 1, 0)))
end
ways

# Calculate the product of the counts at each index
ways = [prod([counts[j][i] for j in eachindex(characters)]) for i in eachindex(unique_combinations)]

ways

=#

#=
prototype code for generate_unique_combinations function

# Generate all combinations of n colors
combinations = vec(join.(Iterators.product([colors for _ in 1:n]...)))

# Function to sort characters in a string
sorted_strings(s) = join(sort(collect(s)))

# Sort each combination
sorted_combinations = map(sorted_strings, combinations) 

# Remove duplicates
unique_combinations = sort(unique(sorted_combinations), rev = true)
=#

#=
prototype code for get_counted_characters function

# Count the number of times each character appears in each combination
counts = [count_character(char, unique_combinations) for char in characters]

# Convert the array of arrays to a matrix and transpose it
counts_matrix = hcat(counts...)
=#
