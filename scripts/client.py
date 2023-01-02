import socket

PORT_NUMBER = 50000
IP_ADDRESS = "192.168.0.21"

# Open a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Set the server address structure
server_address = (IP_ADDRESS, PORT_NUMBER)

# Connect to the server
sock.connect(server_address)

# Main loop
while True:
    # Read a command from the user
    command = input("Enter a command 'forward': ")

    # Send the command to the server
    sock.sendall(bytes(command, 'utf-8'))
    
    if command == "get_data":
        data = sock.recv(1024).decode('utf-8')
        print(data)

# Close the socket
sock.close()
