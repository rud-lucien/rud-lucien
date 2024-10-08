using Sockets

function connect_to_tcp_server(ip, port)
    println("Connecting to $ip:$port...")
    sock = connect(ip, port)
    println("Connected to server.")

    return sock
end

function send_command(sock, command)
    println("Sending command: $command")
    write(sock, command * "\n")  # Send command followed by a newline
    flush(sock)  # Ensure the message is sent
end

function receive_response(sock)
    try
        while isopen(sock)
            # Wait to read data from the server
            response = readline(sock)
            println("Received from server: $response")
        end
    catch e
        println("Error receiving response: $e")
    end
end


function run_tcp_client(ip, port)
    # Connect to the TCP server
    sock = connect_to_tcp_server(ip, port)

    # Start a task to continuously receive responses from the server
    @async receive_response(sock)

    # Send commands to the server
    while true
        println("Enter command to send to server (or type 'exit' to quit):")
        command = readline(stdin)

        if command == "exit"
            break
        else
            send_command(sock, command)
        end
    end

    # Close the socket when done
    close(sock)
    println("Connection closed.")
end

# Example usage: Replace the IP and port with your server's details
run_tcp_client("169.254.0.11", 8080)
