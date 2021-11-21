import subprocess
import socket


HOST = "34.242.114.191"
PORT = 5001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        data = s.recv(1024)
        if data == b'G':
            print("\033[42m")
            subprocess.call("clear")
        elif data == b'g':
            print("\033[32m")
            subprocess.call("clear")
        elif data == b'R':
            print("\033[41m")
            subprocess.call("clear")
        elif data == b'r':
            print("\033[31m")
            subprocess.call("clear")
        elif data == b'z':
            subprocess.call(["paplay", "/usr/share/sounds/speech-dispatcher/test.wav"])
        elif data == b'e':
            s.sendall(bytearray([0,1,2,3,4,5]))
        print('Received', repr(data))

