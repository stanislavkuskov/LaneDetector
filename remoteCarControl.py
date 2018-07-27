import socket

def send_cmd(command, adress = '172.24.1.1', port = 1080):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the CarControl is listening
    server_address = (adress, port)
    sock.connect(server_address)
    try:
        # Send data
        message = command.encode()
        # print(cmd)
        sock.sendall(message)
    finally:
        print('closing socket')
        sock.close()

