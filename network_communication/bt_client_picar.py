import bluetooth
import sys
import tty
import termios

pi_bt_MAC = "E4:5F:01:2C:45:50" # The address of Raspberry PI Bluetooth adapter on the server.
port = 1

def client(server_MAC, port):
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((server_MAC, port))

    print('''Starting picar control over bluetooth
    Key mapping:
    q: quit
    w: forward
    s: backward
    a: turn left
    d: turn right
    other keys: stop''')

    while True:
        key=readkey()

        if key=='q':
            print("quitting")  
            break  
        else:
            sock.send(key)
            print(key)
            data = sock.recv(1024)
            print("server response: ", data.decode("utf-8") )

    sock.close()

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)

if __name__ == '__main__':
    client(pi_bt_MAC, port)