# JuliaCheckCOMPorts

This script checks available COM ports and sends a message to each one to determine which port a device is connected to. It is useful for identifying which COM port a cable is connected to based on the device's response to a specific message.

## Description

The script performs the following steps:

1. Lists all available COM ports.
2. Sends a specified message to each port.
3. Waits for a response from the device.
4. Checks if the response matches the sent message.
5. Identifies and prints the COM port to which the cable is connected.

## Requirements

- Julia 1.0 or higher
- `LibSerialPort` package

## Setup

Ensure you have Julia installed and added to your system PATH.

## Usage

1. Open a terminal or command prompt.
2. Navigate to the directory where your `julia_check_COM_ports` folder is located.
3. Run the script with the project environment activated by typing the following command:
   ```sh
   julia --project=. JuliaCheckCOMPorts.jl