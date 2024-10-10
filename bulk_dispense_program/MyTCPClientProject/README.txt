# TCP Client in Julia

This project contains a simple TCP client implemented in Julia. The client connects to a TCP server, sends commands, and receives responses asynchronously.

## Prerequisites

Ensure that Julia is installed on your system. You can download Julia from the official website: https://julialang.org/downloads/.

## Project Structure

- `MyTCPClientProject/`: Contains the Julia script, along with `Manifest.toml` and `Project.toml` files for environment management.
    - `Julia TCP Client.jl`: The main script for the TCP client.
    - `Manifest.toml`: Defines the exact versions of dependencies.
    - `Project.toml`: Describes the package dependencies and project configuration.

## Getting Started

### 1. Navigate to the script directory

Open a terminal and navigate to the folder where the script is located:

```bash
cd path/to/MyTCPClientProject

For example, if the folder is in your home directory:

bash
Copy code
cd ~/MyTCPClientProject
2. Activate the Julia environment
To ensure that you are working within the correct environment with all necessary dependencies, activate the Julia environment:

Start Julia by typing julia in the terminal.
Inside the Julia REPL (prompt), activate the environment with the following commands:
julia
Copy code
] activate .
This command activates the environment defined by the Project.toml and Manifest.toml files in the MyTCPClientProject folder.

3. Run the TCP client script
While still in the Julia REPL, load the script by running:

julia
Copy code
include("Julia TCP Client.jl")
This will start the TCP client with the default IP and port defined in the script. You can modify these values directly in the script before running if needed.

4. Sending Commands
Once the client is connected to the server, you will be prompted to enter commands. Type your command and press Enter to send it to the server.

To exit the program, simply type exit and press Enter.

Example Session
Here’s how a typical session might look:

bash
Copy code
julia> include("tcp_client.jl")
Connecting to 169.254.0.11:8080...
Connected to server.
Enter command to send to server (or type 'exit' to quit):
5. Close the connection
After you are done sending commands, type exit to close the connection to the server and terminate the program.