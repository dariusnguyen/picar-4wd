import bluetooth
import picar_4wd as fc
import time

pi_bt_MAC="E4:5F:01:2C:45:50" # The address of Raspberry PI Bluetooth adapter on the server. The server might have multiple Bluetooth adapters
port=0

def server(server_MAC, port):
    backlog = 1
    size = 1024
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    s.bind((server_MAC, port))
    s.listen(backlog)
    print("listening on port ", port)
    try:
        client, clientInfo = s.accept()
        while 1:   
            print("key received:", end='')
            data = client.recv(size)
            if data:
                print(data)
                return_msg=parse(data)
                client.send(return_msg) # Return message to client
    except: 
        print("Closing socket")
        client.close()
        s.close()

def parse(data):
    key=data.decode("utf-8")
    if len(key)>1:
        msg='More than 1 key received. Skipping'
    else:
        if key=='w':
            forward()
            msg='Moving forward'
        elif key=='s':
            backward()
            msg='Moving backward'
        elif key=='d':
            turn(-90)
            msg='Turning right 90*'
        elif key=='a':
            turn(90)
            msg='Turning left 90*'
        else:
            fc.stop()
            msg='Stopping'
    return msg

def forward(distance=10):
    power=40
    speed=6 #cm/second
    duration=distance/speed

    # print('forward {} cm'.format(distance))

    fc.forward(power)
    time.sleep(duration)
    
    fc.stop()
    time.sleep(.1)

def backward(distance=10):
    power=40
    speed=6 #cm/second
    duration=distance/speed

    # print('backward {} cm'.format(distance))

    fc.backward(power)
    time.sleep(duration)
    
    fc.stop()
    time.sleep(.1)

def turn(angle):
    duration=1

    print('turn {}*'.format(angle))

    if angle==-45:
        power=67
        fc.turn_right(power)
    elif angle==-90:
        power=90
        fc.turn_right(power)
    elif angle==45:
        power=67
        fc.turn_left(power)
    elif angle==90:
        power=95
        fc.turn_left(power)
    else:
        print('angle {} not accepted'.format(angle))
    time.sleep(duration)
    fc.stop()
    time.sleep(.1)

if __name__ == '__main__':
    server(pi_bt_MAC, port)

