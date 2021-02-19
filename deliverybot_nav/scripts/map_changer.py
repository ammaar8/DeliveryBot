#!/usr/bin/env python3

import tkinter as tk
import subprocess
import signal
import os

ROOT_PATH = os.path.split(os.path.dirname((os.path.abspath(__file__))))[0]

print(ROOT_PATH)
MAPS_DIR = os.path.join(ROOT_PATH, "maps", "building")
floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

child = None

def change_map(floor):
    global child
    if child is not None:
        child.send_signal(signal.SIGINT)
    print("Changing map to " + floors[floor])
    child = subprocess.Popen(["rosrun", "map_server", "map_server", str(os.path.join(MAPS_DIR, floors[floor] + ".yaml"))])
    print("Map to " + floors[floor])


class Application(tk.Frame):
    def __init__(self, master = None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        selected_floor = tk.IntVar()
        self.floor_label = tk.Label(self, text="Floor")
        self.floor_dropdown = tk.OptionMenu(self,selected_floor, 0, 1, 2, command=change_map)

        self.floor_label.pack(side="left")
        self.floor_dropdown.pack(side="left")


if __name__ == "__main__":
    root = tk.Tk()
    change_map(0)
    root.title('Map Controller')
    app = Application(master=root)
    app.mainloop()