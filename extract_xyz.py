#! /usr/bin/env python

# Define the path to your input text file
input_file_path = '/home/Xinyun/amiga_main/amiga_moveit_wrapper/scripts/predef_traj_cir_strai.txt'

# Function to extract position information from the text
def extract_position(line):
    x = float(line.split(': ')[1])
    y = float(next(file).split(': ')[1])
    z = float(next(file).split(': ')[1])
    return x, y, z

# Open and read the input file
with open(input_file_path, 'r') as file:
    i = 0
    while True:
        line = file.readline()
        if not line:
            break
        if line.startswith("position:"):
            x, y, z = extract_position(next(file))
            print(f"i = {i}")
            print(f"position:\n  x: {x}\n  y: {y}\n  z: {z}")
            # Skip the orientation lines
            for _ in range(4):
                next(file)
            print("-------------")
            i += 1
