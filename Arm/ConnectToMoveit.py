import pyads
from time import sleep
import numpy as np
import roslibpy
import time
from math import sin, cos

plc = pyads.Connection('192.168.0.182.1.1', 851)
plc.open()

client = roslibpy.Ros(host='192.168.26.18', port=9090)

angle = [0, 0, 0, 0, 0, 0]
last_angle = [0, 0, 0, 0, 0, 0]


def cal_pos():
    for t in range(100):
        angle[0] = 10*sin(0.02*t) + 2*cos(0.04*t) + \
            3*cos(0.02*t) - 5*cos(0.04*t)
        angle[1] = 10*sin(0.01*t) + 2*cos(0.002*t) + \
            3*cos(0.001*t) - 5*cos(0.02*t)
        angle[2] = 10*sin(0.03*t) + 2*cos(0.06*t) + \
            3*cos(0.03*t) - 5*cos(0.06*t)
        angle[3] = 10*sin(0.02*t) + 2*cos(0.04*t) + \
            3*cos(0.02*t) - 5*cos(0.04*t)
        angle[4] = 10*sin(0.01*t) + 2*cos(0.002*t) + \
            3*cos(0.001*t) - 5*cos(0.02*t)
        angle[5] = 10*sin(0.03*t) + 2*cos(0.06*t) + \
            3*cos(0.03*t) - 5*cos(0.06*t)
        k = [7, 5, 3, 5, 4, 5]
        for i in range(6):
            plc.write_by_name(
                "MAIN.Pos_arr2["+str(t)+","+str(i+1)+"]", angle[i]*k[i])


def click(var_name):
    var_name = "MAIN."+var_name
    trigger = plc.read_by_name(var_name)
    trigger = False
    plc.write_by_name(var_name, trigger)
    sleep(0.01)
    trigger = True
    plc.write_by_name(var_name, trigger)


def move():
    click("FiFo_SetChannelOverride.bExecute")
    click("FiFo_GroupDisintegrate.bExecute")
    click("integrate_do")
    click("FIFO_write_do")
    click("start_do")


def receive_message():
    listener = roslibpy.Topic(client, '/arm', 'std_msgs/Float64MultiArray')
    message_data = [None]
    for i in range(6):
        angle[i] = plc.get_symbol("MAIN.Pos"+str(i+1))
    while 1:
        print(angle)
        listener.subscribe(
            lambda message: message_data.__setitem__(0, message['data']))

        if message_data[0] is not None:
            print(message_data[0])
            index = 0
            for i in message_data[0]:
                if index % 6 == 4 or index % 6 == 3:
                    plc.write_by_name(
                        "MAIN.Pos_arr2["+str(index//6)+","+str(index % 6+1)+"]", -i)
                else:
                    plc.write_by_name(
                        "MAIN.Pos_arr2["+str(index//6)+","+str(index % 6+1)+"]", i)
                index = index + 1

            last_angle = message_data[0][-6:]

            print(index//6)
            plc.write_by_name("MAIN.pos_num", index//6)
            move()
            message_data[0] = None
        # listener.unsubscribe()
        # print(message)


client.run()
receive_message()
