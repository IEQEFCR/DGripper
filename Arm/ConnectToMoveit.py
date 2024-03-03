import pyads
from time import sleep
import numpy as np
import roslibpy
import time
from math import sin, cos
import multiprocessing
import os

plc = pyads.Connection('192.168.0.182.1.1', 851)
plc.open()

client = roslibpy.Ros(host='192.168.137.40', port=9090)

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


def update_joint_state():
    if os.getpid() == 0:
        return
    client.run()
    publisher = roslibpy.Topic(
        client, '/arm_state', 'std_msgs/String')
    publisher.advertise()
    angle_to_send = [0, 0, 0, 0, 0, 0]
    for i in range(6):
        angle[i] = plc.get_symbol("MAIN.Pos"+str(i+1), auto_update=True)
    time.sleep(0.1) 
    while 1:
        for i in range(6):
            angle_to_send[i] = float(str(angle[i].value))*np.pi/180
            if i == 3 or i == 4:
                angle_to_send[i] = -angle_to_send[i]
            angle_to_send[i] = round(angle_to_send[i], 4)
        publisher.publish(roslibpy.Message(
            {'data': str(angle_to_send[0])+','+str(angle_to_send[1])+','+str(angle_to_send[2])+','+str(angle_to_send[3])+','+str(angle_to_send[4])+','+str(angle_to_send[5])}))
        # time.sleep(0.01)


def receive_message():
    if os.getpid() == 1:
        return
    client.run()
    listener = roslibpy.Topic(client, '/arm', 'std_msgs/Float64MultiArray')
    message_data = [None]
    while 1:
        # print("hello")
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


if __name__ == '__main__':
    multiprocessing.Process(target=update_joint_state).start()
    receive_message()
    update_joint_state()
