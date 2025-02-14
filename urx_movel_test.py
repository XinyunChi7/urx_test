import urx
import socket
import time

'''
rob.movej((1, 2, 3, 4, 5, 6), a, v)
rob.movel((x, y, z, rx, ry, rz), a, v)
(rob.movel((0.1, 0, 0, 0, 0, 0), a, v, relative=true)  # move relative to current pose)
NOTE the orientation is always assumed as NOT changing

'''

rob = urx.Robot("10.0.3.7")

rob.set_tcp((0, 0, -0.1, 0, 0, 0))
rob.set_payload(2, (0, 0, 0.1))
print("tcp + payload set")

print ("Current transform is: ",  rob.get_pose())
print ("Current tool tip pose (xyz) is: ",  rob.get_pos())
# print ("Current transformation from tcp to current csys is: ",  rob.getl()) # error: exceed time

# time.sleep(0.2)

# move arm
print("-----move 1------")
target_pose = (0.7343119738351755, -0.10805381296384714, 1.4305678176849255, 0.7070887424652471, 0.7071248192334569, -1.1422450121467834e-05)
rob.movel(target_pose, acc=0.1, vel=0.1)  # Adjust acc and vel as needed


rob.movel((0.7343119738351755, -0.10805381296384714, 1.4305678176849255, 0.7070887424652471, 0.7071248192334569, -1.1422450121467834e-05), 0.01, 0.01)
# rob.movel((x, y, z, rx, ry, rz), a, v)
print ("Current transform is: ",  rob.get_pose())
print ("Current tool tip pose (xyz) is: ",  rob.get_pos())


print("-----move 2------")
rob.movel((0.7355677842257619, -0.10805381296384714, 1.43060728311636, 0.7070887424652471, 0.7071248192334569, -1.1422450121467834e-05), wait=False)
# rob.movel((x, y, z, rx, ry, rz), a, v)
print ("Current transform is: ",  rob.get_pose())
print ("Current tool tip pose (xyz) is: ",  rob.get_pos())


print("-----move 3------")
rob.movel((0.7368186385064617, -0.10805381296384714, 1.430725523658636, 0.7070887424652471, 0.7071248192334569, -1.1422450121467834e-05), wait=False)
# rob.movel((x, y, z, rx, ry, rz), a, v)
print ("Current transform is: ",  rob.get_pose())
print ("Current tool tip pose (xyz) is: ",  rob.get_pos())





# rob.movel((x, y, z, rx, ry, rz), wait=False)
# while True :
#     sleep(0.1)  #sleep first since the robot may not have processed the command yet
#     if rob.is_program_running():
#         break

# rob.movel((x, y, z, rx, ry, rz), wait=False)
# while rob.getForce() < 50:
#     sleep(0.01)
#     if not rob.is_program_running():
#         break
# rob.stopl()

# try:
#     rob.movel((0,0,0.1,0,0,0), relative=True)
# except RobotError.ex:
#     print("Robot could not execute move (emergency stop for example), do something", ex)