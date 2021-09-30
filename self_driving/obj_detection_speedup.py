import modded_detect_picamera as dp
import time
import concurrent.futures
threshold=0.4
path='/home/pi/picar-4wd/my_scripts/data/'
labels = dp.load_labels(path+'coco_labels.txt')
interpreter = dp.Interpreter(path+'detect.tflite')

def main():
    start=time.time()
    objs=dp.scan_once(labels, interpreter, threshold, 0)
    end=time.time()    
    time_per_scan=end-start

    start=time.time()
    for _ in range(5):
        objs=dp.scan_once(labels, interpreter, threshold, 0)
    end=time.time()
    time_5_scans=end-start

    start=time.time()
    with concurrent.futures.ThreadPoolExecutor() as executor:
        _=[executor.submit(dp.scan_once, labels, interpreter, threshold, 0) for _ in range(5)]
    end=time.time()
    time_5_scans_multithread=end-start

    print('time per scan:', time_per_scan)
    print('time for 5 scans:', time_5_scans)
    print('time for 5 scans with multithreading:', time_5_scans_multithread)

if __name__ == "__main__":
    main()