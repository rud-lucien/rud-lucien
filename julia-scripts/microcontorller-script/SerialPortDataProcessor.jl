# Import serial port package
using LibSerialPort
using Dates
# Create a fuction called open_serial_port that takes portname and baudrate as arguments and opens the serial port.
function open_serial_port(portname::String, baudrate::Int)::SerialPort
    try
        port = LibSerialPort.open(portname, baudrate)
        println("Serial port opened successfully!")
        return port
    catch e
        println("Could not open port $portname: $e")  # Error handling
    end
end

# Create a function called close_serial_port that closes the serial port opened by the open_serial_port function.
function close_serial_port(port::SerialPort)::Nothing
    close(port)
    return nothing
end

# Create a function called check_serial_port that checks if the serial port is open or closed. The function should return a boolean value. and print serial port open if its open and serila port not opened if its closed

function check_serial_port(port::SerialPort)::Bool
    if isopen(port)
        println("Serial port open")
        return true
    else
        println("Serial port not open")
        return false
    end
end



#________________________________________________________________________________________________________________________
#Test the fuctions that open and close serial ports
#________________________________________________________________________________________________________________________
# Call open_serial_port function to open the serial port
#data_ch = open_serial_port("COM12", 9600)

# Call check_serial_port function to check if the serial port is open
#check_serial_port(data_ch)

# Call close_serial_port function to close the serial port
#close_serial_port(data_ch)

# Call check_serial_port function to check if the serial port is closed
#check_serial_port(data_ch)


# Create a function called read_serial_data that reads data from the data_ch if its open and returns it as an array of strings, else it notifies the user
function read_serial_data(port::SerialPort)::Channel{Array{SubString{String},1}}
    if check_serial_port(port) == true
        data_ch = Channel{Array{SubString{String},1}}(1)

        @async while true
            data = readuntil(port, '\n')
            data_str = String(data)
            str_values = split(data_str)  # Split the string into an array of substrings
            str_values = str_values[2:end]  # Drop the first element
            #@show str_values
            put!(data_ch, str_values)
            sleep(0.1)  # Pause to prevent high CPU usage
        end

        return data_ch
    else
        println("Serial port not open")
    end
end


#create a function called parse_data that takes an array of strings as an argument and returns an array of floats
function parse_data(data::Array{SubString{String},1})
    # Remove any non-numeric strings
    numeric_data = filter(x -> isdigit(x[1]) || x[1] == '.', data)

    # Convert each string in the array to a float
    float_data = map(x -> parse(Float64, x), numeric_data)

    return float_data
end


function view_data_for_live_plot(incoming_data::Channel{Array{SubString{String},1}})
    start_time = now()
    @async while true
        data = take!(incoming_data)
        parsed_data = parse_data(data)
        elapsed_time = (now() - start_time).value / 1000
        parsed_data = [elapsed_time; parsed_data]
        #@show parsed_data
    end
end

data_ch = open_serial_port("COM12", 9600)

# Call the read_serial_data function
read_serial_data(data_ch)

# Call the parse_live_data function
view_data_for_live_plot(read_serial_data(data_ch))

# Call the close_serial_port function to close the serial port
close_serial_port(data_ch)

# Call check_serial_port function to check if the serial port is closed
check_serial_port(data_ch)