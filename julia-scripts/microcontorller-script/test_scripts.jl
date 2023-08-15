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