import picar_4wd as fc
import time

def main():
    power=68
    # fc.servo.set_angle(90)
    # time.sleep(.5)
    # while (True):
    #     scan_list, angle_list = fc.scan_step(25, 10)
    #     if scan_list is not False:  #if scan_step() returns False then skip remaining statements
    #         break
    #     # time.sleep(1)
    # fc.forward(power)
    # fc.turn_right(power)
    # fc.backward(power)
    fc.turn_left(power)
    time.sleep(1)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
