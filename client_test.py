#!/usr/bin/env python

import socket

receiver_ip = "10.0.3.15"  
receiver_port = 23333

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((receiver_ip, receiver_port))
        print("binded")
        print("Waiting for data...")
        while True:
            data, addr = sock.recvfrom(1024)  # Buffer size of 1024 bytes
            joint_values_str = data.decode()
            joint_values = list(map(float, joint_values_str.split()))
            print("Received joint values:", joint_values)

if __name__ == "__main__":
    main()



# import socket

# def main():
#     client_socket = socket.socket()
#     client_socket.bind(('10.0.0.129', 23333))
#     client_socket.listen()
#     a,b = client_socket.accept()

#     # server_ip = '10.0.3.11'  
#     # server_ip = '10.0.0.129'
#     # server_port = 30000 

#     # client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#     # client_socket.bind(('0.0.0.0', 0))  # Bind 
#     client_socket.settimeout(10.0)  

#     while True:
#         try:
#             data, server_address = client_socket.recvfrom(1024)
#             print("Received from {}: {}".format(server_address, data.decode()))
#         except socket.timeout:
#             print("Timeout occurred while waiting for data.")
#             break

#     client_socket.close()

# if __name__ == "__main__":
#     main()
