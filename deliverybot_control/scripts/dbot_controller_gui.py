#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64
import Tkinter as tk

global ns

rospy.init_node("dbot_controller_gui")

try:
    ns = sys.argv[1]
    rospy.loginfo("Starting with namespace " + ns)
except:
    ns = ""
    rospy.logwarn("No namespace provided!")


door_pub = rospy.Publisher(
    ns + "/door_position_controller/command",
    Float64,
    queue_size=10
)

pusher_pub = rospy.Publisher(
    ns + "/pusher_position_controller/command",
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


class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.btn_open_door = tk.Button(self, text="Open Door", command=open_door)
        self.btn_open_door.pack(side="left")

        self.btn_close_door = tk.Button(self, text="Close Door", command=close_door)
        self.btn_close_door.pack(side="left")

        self.btn_pusher_out = tk.Button(self, text="Pusher Out", command=pusher_out)
        self.btn_pusher_out.pack(side="right")

        self.btn_pusher_in = tk.Button(self, text="Pusher In", command=pusher_in)
        self.btn_pusher_in.pack(side="right")


if __name__ == "__main__":
    root = tk.Tk()
    root.title('DeliveryBot Controller')
    app = Application(master=root)
    app.mainloop()
