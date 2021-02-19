#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import sys
import Tkinter as tk

global ns

rospy.init_node('elevator_teleop')

door = {
    "CLOSED": 0,
    "OPEN": 1,
}

floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

try:
    ns = sys.argv[1]
except:
    ns = ""    
rospy.loginfo("Namespace := " + ns)

floor_door_left_pubs = []
floor_door_right_pubs = []

for i in range(len(floors.keys())):
    floor_door_left_pubs.append(
        rospy.Publisher(
        ns + '/' + str(floors[i]) + '_door_left_controller/command',
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        ns + '/' + str(floors[i]) + '_door_right_controller/command',
        Float64,
        queue_size=10)
    )

# Publishers
floor_pub = rospy.Publisher(
    ns + '/floor_position_controller/command',
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    ns + '/car_door_left_controller/command',
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    ns + '/car_door_right_controller/command',
    Float64,
    queue_size=10
)



DOOR_OPEN = 0.5

FLOOR = 0
DOOR = door["CLOSED"]


def go_to_floor(floor):
    global FLOOR, DOOR
    rospy.loginfo("Going to " + str(floor) + " floor")
    FLOOR = int(floor)
    if DOOR != door["CLOSED"]:
        close_doors()

    floor_pub.publish(4.0 * floor)

def open_doors():
    global DOOR

    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))

    
    floor_door_right_pubs[FLOOR].publish(DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-DOOR_OPEN)

    pub_car_right.publish(DOOR_OPEN)
    pub_car_left.publish(-DOOR_OPEN)

    DOOR = door["OPEN"]

def close_doors():
    global DOOR
    rospy.loginfo("CLOSING DOORS")

    
    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)

    pub_car_right.publish(0)
    pub_car_left.publish(0)
    
    DOOR = door["CLOSED"]


class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        selected_floor = tk.IntVar()
        self.floor_label = tk.Label(self, text="Floor")
        self.floor_label.pack(side="left")

        self.dropdown_floors = tk.OptionMenu(self, selected_floor, 0, 1, 2, command=go_to_floor)
        self.dropdown_floors.pack(side="left")

        self.btn_open_doors = tk.Button(self, text="Open Doors", command = open_doors)
        self.btn_open_doors.pack(side="bottom")
        
        self.btn_close_doors = tk.Button(self, text="Close Doors", command = close_doors)
        self.btn_close_doors.pack(side="bottom")        
        


if __name__== "__main__":
    root = tk.Tk()
    root.title('Elevator Controller')
    app = Application(master=root)
    app.mainloop()
