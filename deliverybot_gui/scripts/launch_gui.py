#!/usr/bin/env python

import rospy
import Tkinter as tk
from std_msgs.msg import Float64
import sys
import os
import signal
import subprocess

rospy.init_node("dbot_controller_gui")


# Bot Controls
door_pub = rospy.Publisher(
    "/dbot/door_position_controller/command",
    Float64,
    queue_size=10
)


pusher_pub = rospy.Publisher(
    "/dbot/pusher_position_controller/command",
    Float64,
    queue_size=10
)


DOOR_OPEN = 0.55
DOOR_CLOSED = -1.57075
PUSHER_OUT = 0.24
PUSHER_IN = 0.0


def open_door():
    door_pub.publish(DOOR_OPEN)


def close_door():
    door_pub.publish(DOOR_CLOSED)


def pusher_out():
    pusher_pub.publish(PUSHER_OUT)


def pusher_in():
    pusher_pub.publish(PUSHER_IN)


# Elevator Controls

elevator_door = {
    "CLOSED": 0,
    "OPEN": 1,
}

floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

floor_door_left_pubs = []
floor_door_right_pubs = []

for i in range(len(floors.keys())):
    floor_door_left_pubs.append(
        rospy.Publisher(
        '/elevator/' + str(floors[i]) + '_door_left_controller/command',
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        '/elevator/' + str(floors[i]) + '_door_right_controller/command',
        Float64,
        queue_size=10)
    )

floor_pub = rospy.Publisher(
    '/elevator/floor_position_controller/command',
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    '/elevator/car_door_left_controller/command',
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    '/elevator/car_door_right_controller/command',
    Float64,
    queue_size=10
)

ELEVATOR_DOOR_OPEN = 0.5
FLOOR = 0
ELEVATOR_DOOR = elevator_door["CLOSED"]


def go_to_floor(floor):
    global FLOOR, ELEVATOR_DOOR
    rospy.loginfo("Going to " + str(floor) + " floor")
    FLOOR = int(floor)
    if ELEVATOR_DOOR != elevator_door["CLOSED"]:
        close_doors()

    floor_pub.publish(4.0 * floor)


def open_doors():
    global ELEVATOR_DOOR

    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))

    
    floor_door_right_pubs[FLOOR].publish(ELEVATOR_DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-ELEVATOR_DOOR_OPEN)

    pub_car_right.publish(ELEVATOR_DOOR_OPEN)
    pub_car_left.publish(-ELEVATOR_DOOR_OPEN)

    ELEVATOR_DOOR = elevator_door["OPEN"]


def close_doors():
    global ELEVATOR_DOOR
    rospy.loginfo("CLOSING DOORS")

    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)

    pub_car_right.publish(0)
    pub_car_left.publish(0)
    
    ELEVATOR_DOOR = elevator_door["CLOSED"]

# Floor controller

ROOT_PATH = os.path.split(os.path.split(os.path.dirname((os.path.abspath(__file__))))[0])[0]
MAPS_DIR = os.path.join(ROOT_PATH, "deliverybot", "maps", "building")
floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

term = None

def change_map(floor):
    global term
    if term is not None:
        term.send_signal(signal.SIGINT)
    print("Changing map to " + floors[floor])
    term = subprocess.Popen(["rosrun", "map_server", "map_server", str(os.path.join(MAPS_DIR, floors[floor] + ".yaml"))])
    print("Map to " + floors[floor])


class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()


    def create_widgets(self):
        self.create_bot_widgets()
        self.create_elevator_widgets()
        self.create_map_widgets()


    def create_bot_widgets(self):
        dbot_control_frame = tk.LabelFrame(self, text="DeliveryBot Controls")
        dbot_control_frame.grid_rowconfigure(0, weight=1)
        dbot_control_frame.grid_columnconfigure(0, weight=1)

        dbot_control_frame.pack(fill='x')
        self.btn_open_door = tk.Button(dbot_control_frame, text="Open Door", command=open_door)
        self.btn_open_door.grid(row=0,column=0, sticky="ew")

        self.btn_close_door = tk.Button(dbot_control_frame, text="Close Door", command=close_door)
        self.btn_close_door.grid(row=1,column=0, sticky="ew")

        self.btn_pusher_out = tk.Button(dbot_control_frame, text="Pusher Out", command=pusher_out)
        self.btn_pusher_out.grid(row=0,column=1, sticky="ew")

        self.btn_pusher_in = tk.Button(dbot_control_frame, text="Pusher In", command=pusher_in)
        self.btn_pusher_in.grid(row=1,column=1,sticky="ew")        


    def create_elevator_widgets(self):
        elevator_control_frame = tk.LabelFrame(self, text="Elevator Controls")
        elevator_control_frame.grid_rowconfigure(0, weight=1)
        elevator_control_frame.grid_columnconfigure(0, weight=1)
        elevator_control_frame.pack(fill="x")
        selected_floor = tk.IntVar()

        self.floor_label = tk.Label(elevator_control_frame, text="Floor")
        self.floor_label.grid(row=0, column=0, sticky="ew")

        self.dropdown_floors = tk.OptionMenu(elevator_control_frame, selected_floor, 0, 1, 2, command=go_to_floor)
        self.dropdown_floors.grid(row=1, column=0, sticky="ew")

        self.btn_open_doors = tk.Button(elevator_control_frame, text="Open Doors", command = open_doors)
        self.btn_open_doors.grid(row=0, column=1, sticky="ew")
        
        self.btn_close_doors = tk.Button(elevator_control_frame, text="Close Doors", command = close_doors)
        self.btn_close_doors.grid(row=1, column=1, sticky="ew")        

    def create_map_widgets(self):
        map_widgets_frame = tk.LabelFrame(self, text="Map Control")
        map_widgets_frame.pack(fill='x')
        map_widgets_frame.grid_rowconfigure(0, weight=1)
        map_widgets_frame.grid_columnconfigure(0, weight=1)
        
        selected_floor = tk.IntVar()
        self.floor_label = tk.Label(map_widgets_frame, text="Map floor")
        self.floor_dropdown = tk.OptionMenu(map_widgets_frame, selected_floor, 0, 1, 2, command=change_map)

        self.floor_label.grid(row=0, column=0, sticky="ew")
        self.floor_dropdown.grid(row=0, column=1, sticky="ew")


if __name__ == "__main__":
    root = tk.Tk()
    root.title("DeliveryBot Controller")
    root.resizable(False, False)
	change_map(0)
    app = Application(master=root)
    app.mainloop()
