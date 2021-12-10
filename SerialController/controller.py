import serial
import time
import keyboard  # using module keyboard
import socket

arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.1)

prev_line = ""

def print_pressed_keys(e):
    global prev_line
    line = ', '.join(str(code) for code in keyboard._pressed_events)
    print('\r' + line + ' '*40, end='')
    movement_dict = {
        "linear_x" : 0,
        "linear_y": 0,
        "angular_z" : 0
    }

    for code in keyboard._pressed_events:
        if code == 16:
            movement_dict["angular_z"] = 0.05
        elif code == 18:
            movement_dict["angular_z"] = -0.05
        elif code == 17:
            movement_dict["linear_x"] = 15.0
        elif code == 31:
            movement_dict["linear_x"] = -15.0
        elif code == 30:
            movement_dict["linear_y"] = -15.0
        elif code == 32:
            movement_dict["linear_y"] = 15.0
    
    if prev_line != line:
        arduino.write(bytes('{},{},{}\n'.format(movement_dict["linear_x"], movement_dict["linear_y"], movement_dict["angular_z"]), 'utf-8'))
        prev_line = line
        print("send")

	
keyboard.hook(print_pressed_keys)
keyboard.wait() 