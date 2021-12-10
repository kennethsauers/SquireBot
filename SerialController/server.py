# Echo server program
import socket
import serial

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port

arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.1)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            try:
                data = conn.recv(1024)
                if not data: break
                conn.sendall(b'Message Sent')
                arduino.write(data)
            except Exception as e:
                print(e)
                arduino.write(b'0,0,0\n')
