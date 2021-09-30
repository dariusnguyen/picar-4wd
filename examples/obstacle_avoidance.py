import picar_4wd as fc

speed = 30

def main():
    while True:
        scan_list = fc.scan_step(25, 10) #added ref2 due to modification in init.py
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)
        if tmp != [2,2,2,2]:
            fc.turn_right(speed)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
