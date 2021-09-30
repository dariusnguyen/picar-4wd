import picar_4wd as fc
import time

def main():
    # step=0
    while True:
        step=int(input('Enter angle to rotate, 888 to quit: '))
        if step==888:
            break
        print('Excuting test...')

        reps=5
        duration_list=[]

        for i in range(reps):
            fc.servo.set_angle(0)
            time.sleep(0.1)

            start=time.time()
            fc.servo.set_angle(step)
            end=time.time()
            
            time.sleep(1)
            duration_list.append(end-start)

        avg_duration=sum(duration_list)/len(duration_list)
        print(duration_list)
        print('It took {} sec to turn from 0 to {} on average'.format(avg_duration, step))

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
