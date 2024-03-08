#!/usr/bin/env python3
import os
from getkey import getch

bag_path = './rosbag'

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.exists(bag_path):
        os.makedirs(bag_path)

    while True:
        print("\033c")
        print("Press 's' to start recording, 'q' to quit")
        key = getch()
        if key == 's':
            print("Recording...")
            print("Bag will be saved to"+os.getcwd())
            os.system("rosbag record -a"+" -O "+bag_path+"/rosbag_"+str(len(os.listdir(bag_path)))+".bag")
            
        if key == 'q':
            break
        #clear the output
