#!/usr/bin/env python

import Tkinter as tk
import rospkg
import roslaunch
import rospy
import os
import geometry_msgs.msg
# import tf2_ros
# import tf_conversions
import yaml
import subprocess

    
class MappingGUI(tk.Frame):


    def __init__(self, master=None):
        tk.Frame.__init__(self, master=master)
        self.master = master
        self.BUILDING_NAME = None
        self.map_server_node = None
        self.rospack = rospkg.RosPack()
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.PKG_DIR = self.rospack.get_path('deliverybot_mapping')
        self.MAPS_DIR = os.path.join(self.PKG_DIR, 'maps')
        self.LAUNCH_DIR = os.path.join(self.PKG_DIR, 'launch')

        self.map = {
            "building": {
                "name": None,
                "scheme": None,
                "floors": None,
            },
            "lobby":{
                "pickup": {
                    "x": None,
                    "y": None,
                    "a": None
                }
            },
            "rooms":{
                
            }
        }
        self.pack(expand=True, fill="both")
        self.create_mapping_widgets()
        self.create_details_widgets()


    def create_map_dir(self, building_name):
        self.BUILDING_NAME = building_name
        self.map["building"]["name"] = building_name
        if os.path.isdir(os.path.join(self.MAPS_DIR, building_name)):
            with open(os.path.join(self.MAPS_DIR, building_name, 'building.yaml'), 'r') as f:
                self.map = yaml.safe_load(f)
            rospy.loginfo("Building " + building_name + " loaded.")                                                
        else:
            os.mkdir(os.path.join(self.MAPS_DIR, building_name))
            with open(os.path.join(self.MAPS_DIR, building_name, 'building.yaml'), 'w') as f:
                yaml.dump(self.map, f)
            rospy.loginfo("Building " + building_name + " created.")                


    def start_map_server(self):
        # rospy.init_node('mapping_map_server', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.map_server_node = roslaunch.parent.ROSLaunchParent(
            uuid,
            [
                os.path.join(self.LAUNCH_DIR, "map_server.launch")
            ]
        )
        self.map_server_node.start()
        rospy.loginfo("Map Server started")


    def kill_map_server(self):
        self.map_server_node.shutdown()
        self.map_server_node = None
        rospy.loginfo("Map Server shutdown")


    def save_map_lobby(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [
            os.path.join(self.LAUNCH_DIR, "map_saver_lobby.launch"),
            str("location:=" + os.path.join(self.MAPS_DIR, self.BUILDING_NAME))
            ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (
                roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                roslaunch_args
            )
        ]
        map_saver_lobby = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        map_saver_lobby.start()
        rospy.loginfo("Lobby map saved.")


    def save_map_floor(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [
            os.path.join(self.LAUNCH_DIR, "map_saver_floor.launch"),
            str("location:=" + os.path.join(self.MAPS_DIR, self.BUILDING_NAME))
            ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [
            (
                roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                roslaunch_args
            )
        ]
        map_saver_lobby = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        map_saver_lobby.start()
        rospy.loginfo("Floor map saved.")


    def add_naming_scheme(self):
        pass


    def add_room_coods(self, room_numebr):
        print("Marked Room", room_numebr)
        # trans = self.tfBuffer.lookup_transform(
        #     '/map',
        #     '/dbot/base_link',
        #     rospy.Time()
        # )
        # print(
        #     "x: ", trans.transform.translation.x, '\n',
        #     "y: ", trans.transform.translation.y, '\n',
            # "a: ", tf_conversions.transformations.euler_from_quaternion(
            #     [
            #         trans.transform.rotation.x,
            #         trans.transform.rotation.y,
            #         trans.transform.rotation.z,
            #         trans.transform.rotation.w,
            #     ]
            # )
        # )


    def add_floor_count(self):
        pass


    def save_building_details(self):
        pass


    def add_elevator_in(self):
        pass


    def add_elevator_out(self):
        pass


    def create_mapping_widgets(self):
        self.var_room_number = tk.StringVar()
        self.var_building_name = tk.StringVar()

        def add_room_coods_helper():
            if (self.var_room_number.get() != ""):
                self.add_room_coods(self.var_room_number.get())
                room_entry.delete(0, tk.END)

        mapping_frame = tk.LabelFrame(self, text="Mapping")
        mapping_frame.grid_columnconfigure(0, weight=1, uniform="something")
        mapping_frame.grid_rowconfigure(0, weight=1, uniform="something")
        mapping_frame.pack(side="top", expand=True, fill="both")

        building_name_label = tk.Label(mapping_frame, text="Bldg Name")
        building_name_label.grid(column=0, row=0, sticky="ew")
        building_name_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_building_name)
        building_name_entry.grid(column=1, row=0, sticky="ew")
        building_name_btn = tk.Button(mapping_frame, text="Create", command=lambda: self.create_map_dir(self.var_building_name.get()))
        building_name_btn.grid(column=2, row=0, sticky="EW")

        label_server = tk.Label(mapping_frame, text="Server")
        label_server.grid(column=0, row=1, sticky="EW")
        start_btn = tk.Button(mapping_frame, text="Start", command=self.start_map_server)
        start_btn.grid(column=1, row=1, sticky="EW")
        clear_btn = tk.Button(mapping_frame, text="Clear", command=self.kill_map_server)
        clear_btn.grid(column=2, row=1, sticky="EW")

        label_save = tk.Label(mapping_frame, text="Save")
        label_save.grid(column=0, row=2, sticky="EW")
        lobby_btn = tk.Button(mapping_frame, text="Lobby", command=self.save_map_lobby)
        lobby_btn.grid(column=1, row=2, sticky="EW")
        floor_btn = tk.Button(mapping_frame, text="Floor", command=self.save_map_floor)
        floor_btn.grid(column=2, row=2, sticky="EW")

        label_elevator = tk.Label(mapping_frame, text="Elevator")
        label_elevator.grid(column=0, row=3, sticky="EW")
        el_out_btn = tk.Button(mapping_frame, text="EL Out")
        el_out_btn.grid(column=1, row=3, sticky="EW")
        el_in_btn = tk.Button(mapping_frame, text="EL In")
        el_in_btn.grid(column=2, row=3, sticky="EW")

        label_room = tk.Label(mapping_frame, text="Room")
        label_room.grid(column=0, row=4, sticky="EW")
        room_entry = tk.Entry(mapping_frame, bd=2, textvariable=self.var_room_number, width=4)
        room_entry.grid(column=1, row=4, sticky="EW")
        mark_btn = tk.Button(mapping_frame, text="Mark", command=add_room_coods_helper)
        mark_btn.grid(column=2, row=4, sticky="EW")


    def create_details_widgets(self):
        '''
        TODO - Add functionality for wings. For now just include the wing name in the building name.
        Numbering Variabel
        F <- Floor Number
        R <- Room Number
        Example - F0R <- Numbering scheme has an extra 0 in between floor and room number
        '''
        self.var_wing_name = tk.StringVar()
        self.var_floors = tk.StringVar()
        self.var_numbering_format = tk.StringVar()

        map_details_frame = tk.LabelFrame(self, text="Map Details")
        map_details_frame.grid_columnconfigure(0, weight=1, uniform="something")
        map_details_frame.grid_rowconfigure(0, weight=1, uniform="something")
        map_details_frame.pack(side="top", expand=True, fill="both")

        building_floors_label = tk.Label(map_details_frame, text="Floors")
        building_floors_label.grid(column=0, row=1, sticky="ew")
        building_floors_entry = tk.Entry(map_details_frame, bd=2, textvariable=self.var_floors)
        building_floors_entry.grid(column=1, row=1, sticky="ew")

        building_scheme_label = tk.Label(map_details_frame, text="Scheme")
        building_scheme_label.grid(column=0, row=2, sticky="ew")
        building_scheme_entry = tk.Entry(map_details_frame, bd=2, textvariable=self.var_numbering_format)
        building_scheme_entry.grid(column=1, row=2, sticky="ew")

        save_details_btn = tk.Button(map_details_frame, text="Complete Map")
        save_details_btn.grid(column=0, columnspan=2, row=3, sticky="ew")        


if __name__ == "__main__":
    rospy.init_node("dbot_mapping", anonymous=True)
    root = tk.Tk()
    root.resizable(False, False)
    app = MappingGUI(master=root)
    tk.mainloop()