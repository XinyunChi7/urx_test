import urx
import socket
import time

rob = urx.Robot("10.0.3.7")

# Set robot's TCP and payload
rob.set_tcp((0, 0, -0.1, 0, 0, 0))
rob.set_payload(2, (0, 0, 0.1))

print("TCP and payload set")
# time.sleep(0.2)

print("Current transform is:", rob.get_pose())
print("Current tool tip pose (xyz) is:", rob.get_pos())

# Create a socket to receive data from the sender
server_ip = "10.0.3.11"  
server_port = 23333 # Replace!!
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((server_ip, server_port))
    print("Socket bound successfully.")
except OSError as e:
    print("Error binding socket:", e)

# Wait for data and extract the 6 joint state numbers
print("Waiting for data...")
data, addr = sock.recvfrom(1024)  # set buffer size
numbers = [float(num_str) for num_str in data.decode().split()]

print(numbers)

rob.movej(numbers, 0.01, 0.01)
print("Robot moved")


print("Current transform is:", rob.get_pose())
print("Current tool tip pose (xyz) is:", rob.get_pos())

# Close the socket
sock.close()



# Move the robot using the received numbers
# print("-----move 1------")
# rob.movej((-1.4749778, -2.72707696, 0.95207711, -1.3646729, 1.4746287, -2.35584542), 0.01, 0.01)



