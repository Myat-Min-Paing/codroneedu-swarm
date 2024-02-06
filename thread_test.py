from threading import Thread
import time

done = False

def worker(text):
    counter = 0
    while True:
        time.sleep(1)
        counter += 1
        print(f"{text} : {counter}")
        
t1 = Thread(target=worker, daemon=True, args=("ABC",))
t2 = Thread(target=worker, daemon=False, args=("XYZ",))

t1.start()
t2.start()

t1.join()
t2.join()