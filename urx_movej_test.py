import urx
import socket
import time


'''
RUN 'python urx_js_socket.py' 
to receive real-time updated joints states using IP socket,
and send movej command to the robot
'''


rob = urx.Robot("10.0.3.7")
rob.set_tcp((0, 0, -0.1, 0, 0, 0))
rob.set_payload(2, (0, 0, 0.1))

print("TCP and payload set")

server_ip = "127.0.0.1"
server_port = 55561

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))
print("Connected to server on {}:{}".format(server_ip, server_port))

try:
    while True:
        data = sock.recv(1024)
        if not data:
            print("Connection closed by server.")
            break

        received_data = data.decode()
        print("Received:", received_data)

        data_sets = received_data.split('\n')  # Separate data 

        for data_set in data_sets:
            data_list = data_set.split()  # Split the received string using space as the delimiter
            if len(data_list) == 6:  # Ensure complete data set
                numbers = [float(num_str) for num_str in data_list]
                print("Received numbers:", numbers)
                if len(numbers) == 6:
                    # rob.movej(numbers, acc=0.01, vel=0.01)
                    rob.movej(numbers, wait=False, acc=0.01, vel=0.01)
                    print("Robot moved")

                    print("Current transform is:", rob.get_pose())
                    print("Current tool tip pose (xyz) is:", rob.get_pos())
                    print("----------------")

                else:
                    print("Incomplete data set received")
            else:
                print("Invalid data set received")

        # # Single joitn state receive test: Split using a comma and space as a delimiter
        # data_sets = received_data.split(", ")

        # for data_set in data_sets:
        #     numbers = [float(num_str) for num_str in data_set.split()]
        #     print("Received numbers:", numbers)

        # # Send the command to the robot to move using movej
        # rob.movej(numbers, acc=0.01, vel=0.01)
        # print("Robot moved")

        # print("Current transform is:", rob.get_pose())
        # print("Current tool tip pose (xyz) is:", rob.get_pos())
        # print("----------------")





except KeyboardInterrupt:
    pass
finally:
    sock.close()
    rob.close()
