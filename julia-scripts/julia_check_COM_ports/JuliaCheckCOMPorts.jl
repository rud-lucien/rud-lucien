using LibSerialPort

function get_available_ports()
    ports = LibSerialPort.get_port_list()
    if length(ports) == 0
        println("No serial ports found.")
    else
        println("Available serial ports:")
        for port in ports
            println(port)
        end
    end
    return ports
end

function send_and_receive_message(port_name::String, message::String)
    port = LibSerialPort.open(port_name, 9600; mode=LibSerialPort.SP_MODE_READ_WRITE)
    try
        write(port, message)
        sleep(1) # Adjust the sleep duration as necessary
        response = read(port, String)
        if !isempty(response)
            received_message = strip(response)
            if received_message == message
                println("Device found on port $port_name with matching message: $received_message")
                return true
            else
                println("Device on port $port_name responded with a different message: $received_message")
            end
        else
            println("No response received from port $port_name.")
        end
    catch e
        println("Error on port $port_name: $e")
    finally
        close(port)
    end
    return false
end

# function find_connected_device()
#     # Define the message to be sent
#     message = "Hello, COM Port!"
#     ports = get_available_ports()
#     for port_name in ports
#         if send_and_receive_message(port_name, message)
#             println("Cable is connected to port: ", port_name)
#             return port_name
#         end
#     end
#     println("No device with the matching message found on any port.")
#     return nothing
# end

function find_connected_device()
    # Define the message to be sent
    message = "Hello, COM Port!"
    ports = get_available_ports()
    for port_name in ports
        if port_name == "COM1" || port_name == "COM3"
            println("Skipping port $port_name as it is a standard system port.")
            continue
        end
        if send_and_receive_message(port_name, message)
            println("Cable is connected to port: ", port_name)
            return port_name
        end
    end
    println("No device with the matching message found on any port.")
    return nothing
end


# Find the connected device
find_connected_device()