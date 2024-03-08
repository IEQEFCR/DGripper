#!/usr/bin/env python3
import os
from getkey import getch

if __name__ == "__main__":
    while True:
        print("\033c")
        print("Press 's' to start recording, 'q' to quit")
        key = getch()
        if key == 's':
            print("Recording...")
            print("Bag will be saved to"+os.getcwd())
            os.system("rosbag record -a")
            
        if key == 'q':
            break
        #clear the output
