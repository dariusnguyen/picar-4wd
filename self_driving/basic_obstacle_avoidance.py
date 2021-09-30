import picar_4wd as fc
# from importlib import reload
# fc=reload(picar_4wd)

import time
import modded_detect_picamera as dp

def main():
    power = 30
    turn_power=60

    just_turned='no'
    sleep_time_1=.5

    while True:
        scan_list, angle_list = fc.scan_step(25, 10)
        if not scan_list:  #if scan_step() returns False then skip remaining statements
            continue

        print(scan_list, scan_list[3:8], end=' ')
        if 0 in scan_list:
            fc.stop()
            time.sleep(sleep_time_1)
            fc.backward(turn_power)
            time.sleep(sleep_time_1)
            just_turned='no'
            print('backward. ', end='')
            if 0 in scan_list[0:4]:
                fc.turn_right(turn_power)
                just_turned='right'
                print('turn right. ', end='')
                time.sleep(sleep_time_1)
            elif 0 in scan_list[7:11]:
                fc.turn_left(turn_power)
                just_turned='left'
                print('turn left. ', end='')
                time.sleep(sleep_time_1)  
        elif scan_list[3:8] != [2,2,2,2,2]:
            if max(scan_list)<2:
                print('turn around. ', end='')
                fc.turn_right(turn_power)
                time.sleep(4)
            elif just_turned=='right':
                fc.turn_right(turn_power)
                print('turn right. ', end='') 
            elif just_turned=='left':
                fc.turn_left(turn_power)
                print('turn left. ', end='') 
            elif sum(scan_list[0:6])<sum(scan_list[5:11]):
                fc.turn_right(turn_power)
                just_turned='right'
                print('turn right. ', end='') 
            else:
                fc.turn_left(turn_power)
                just_turned='left'
                print('turn left. ', end='') 
        else:
            fc.forward(power)
            time.sleep(sleep_time_1)
            just_turned='no'
            print('forward. ', end='') 
        
        threshold=0.4
        path='/home/pi/picar-4wd/my_scripts/data/'
        labels = dp.load_labels(path+'coco_labels.txt')
        interpreter = dp.Interpreter(path+'detect.tflite')

        dp.scan_once(labels, interpreter, threshold, 0)
        
        time.sleep(sleep_time_1)
        
if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
