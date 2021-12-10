import time
import keyboard  # using module keyboard
import socket

#arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.1)

linear_vel = 30
HOST = '192.168.0.105'    # The remote host
PORT = 50007              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

prev_line = ""

def print_pressed_keys(e):
    global prev_line
    global linear_vel
    
    line = ', '.join(str(code) for code in keyboard._pressed_events)
    print('\r' + line + ' '*40, end='')
    movement_dict = {
        "linear_x" : 0,
        "linear_y": 0,
        "angular_z" : 0
    }

    for code in keyboard._pressed_events:
        if code == 16:
            movement_dict["angular_z"] = -0.2
        elif code == 18:
            movement_dict["angular_z"] = 0.2
        elif code == 17:
            movement_dict["linear_x"] = -linear_vel
        elif code == 31:
            movement_dict["linear_x"] = linear_vel
        elif code == 30:
            movement_dict["linear_y"] = linear_vel
        elif code == 32:
            movement_dict["linear_y"] = -linear_vel
    
    if line != prev_line:
        s.sendall(bytes('{},{},{}\n'.format(movement_dict["linear_x"], movement_dict["linear_y"], movement_dict["angular_z"]), 'utf-8'))
        print(s.recv(1024))
        prev_line = line
	
keyboard.hook(print_pressed_keys)
keyboard.wait() 
