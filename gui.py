import asyncio
import queue
import sys
import threading
import tkinter as tk
import tkinter.font as tkFont
import tkinter.messagebox

from bleak.exc import BleakBluetoothNotAvailableError

from controller import Controller
from discoverer import start_discoverer
from config import get_resource
from virtual_controller import VirtualController
import cemuhookserver

controller_frame_size = 200
battery_height = 40

background_color = "#aaaaaa"
block_color = "#404040"
player_number_bg_color = "#8B8B8B"

CONTROLLER_UPDATED_EVENT = '<<ControllersUpdated>>'


class PlayerInfoBlock:
    def __init__(self, parent):
        self.parent = parent
        self.controller_label = None
        self.player_led_label = None

        self.load_pictures()
        self.init_interface()

    def init_interface(self):
        self.main_frame = tk.Frame(self.parent, width=controller_frame_size, height=controller_frame_size + 8 + 40,
                                   bg=player_number_bg_color)
        self.main_frame.pack(padx=10, pady=10, side=tk.LEFT)
        self.main_frame.pack_propagate(False)

        self.controllers_frame = tk.Frame(self.main_frame, width=controller_frame_size,
                                          height=controller_frame_size - battery_height, bg=block_color)
        self.controllers_frame.pack()
        self.controllers_frame.pack_propagate(False)

        self.battery_frame = tk.Frame(self.main_frame, width=controller_frame_size, height=battery_height,
                                      bg=block_color, padx=50)
        self.battery_frame.pack()
        self.battery_frame.pack_propagate(False)

    def load_pictures(self):
        self.joycon2leftandright = tk.PhotoImage(file=get_resource("images/joycon2leftandright.png"))
        self.joycon2right_sideway = tk.PhotoImage(file=get_resource("images/joycon2right_sideway.png"))
        self.joycon2left_sideway = tk.PhotoImage(file=get_resource("images/joycon2left_sideway.png"))
        self.procontroller2 = tk.PhotoImage(file=get_resource("images/procontroller2.png"))
        self.battery_h = tk.PhotoImage(file=get_resource("images/battery_h.png"))
        self.battery_m = tk.PhotoImage(file=get_resource("images/battery_m.png"))
        self.battery_l = tk.PhotoImage(file=get_resource("images/battery_l.png"))
        self.player_leds = {nb: tk.PhotoImage(file=get_resource(f"images/player{nb}.png")) for nb in range(1, 5)}

    def clearControllerInfo(self):
        if self.controller_label is not None:
            self.controller_label.destroy()
            self.controller_label = None

        if self.player_led_label is not None:
            self.player_led_label.destroy()
            self.player_led_label = None

    def get_image_for_battery_level(self, controller: Controller):
        if controller.battery_voltage > 3.25:
            return self.battery_h
        if controller.battery_voltage > 3.125:
            return self.battery_m
        return self.battery_l

    def displayControllersInfo(self, virtualController: VirtualController):

        if not virtualController.is_single():
            image = self.joycon2leftandright
        elif virtualController.is_single_joycon_right():
            image = self.joycon2right_sideway
        elif virtualController.is_single_joycon_left():
            image = self.joycon2left_sideway
        else:
            image = self.procontroller2

        self.controller_label = tk.Label(self.controllers_frame, image=image, bg=block_color)
        self.controller_label.pack(fill="none", expand=True)

        # Battery levels
        if virtualController.is_single():
            # 1 controller
            self.battery_label = tk.Label(self.battery_frame,
                                          image=self.get_image_for_battery_level(virtualController.controllers[0]),
                                          bg=block_color)
            self.battery_label.pack()
        else:
            # 2 controllers
            self.battery_label = tk.Label(self.battery_frame,
                                          image=self.get_image_for_battery_level(virtualController.controllers[0]),
                                          bg=block_color)
            self.battery_label.pack(side='left')
            self.battery_label2 = tk.Label(self.battery_frame,
                                           image=self.get_image_for_battery_level(virtualController.controllers[1]),
                                           bg=block_color)
            self.battery_label2.pack(side='right')

        self.player_led_label = tk.Label(self.main_frame, image=self.player_leds[virtualController.player_number],
                                         bg=player_number_bg_color)
        self.player_led_label.pack(pady=20)


class ControllerWindow:
    def __init__(self):
        self.root = None
        self.main_frame = None
        self.no_controllers = True
        self.message_queue = queue.Queue()
        self.quit_event = threading.Event()

        global t_
        t_ = threading.Thread(target=cemuhookserver.run_server, daemon=True)
        t_.start()

    def init_interface(self):
        self.root = tk.Tk()
        photo = tk.PhotoImage(file=get_resource('images/icon.png'))
        self.root.wm_iconphoto(False, photo)
        self.root.title("Switch 2 Controller Tool")
        self.root.geometry("1000x400+50+50")
        self.root.minsize(1000, 400)
        self.root.config(bg=background_color, padx=10, pady=10)
        self.font = tkFont.Font(family="Arial", size=16, weight="bold")
        self.pairing_hint_image = tk.PhotoImage(file=get_resource("images/pairing_hint.png"))

        self.update([None])

    def update(self, controllers_info):
        self.no_controllers = all(c is None for c in controllers_info)

        if self.main_frame is not None:
            self.main_frame.destroy()

        self.main_frame = tk.Frame(self.root, bg=background_color)
        self.main_frame.pack(pady=50, fill=tk.Y)

        if self.no_controllers:
            tk.Label(self.main_frame, text="Press button of a paired controller, or hold sync button to pair",
                     font=self.font, bg=background_color).pack()
            pairing_hint = tk.Label(self.main_frame, image=self.pairing_hint_image, bg=background_color)
            pairing_hint.pack(pady=10)
        else:
            self.players_info = [PlayerInfoBlock(self.main_frame) for i in range(4)]

            for i, player_info in enumerate(self.players_info):
                controller_info = controllers_info[i]
                if controller_info is not None:
                    player_info.displayControllersInfo(controller_info)

    def start(self):
        def update_controllers_callback_threadsafe(controllers: list[VirtualController]):
            self.message_queue.put(controllers)
            self.root.event_generate(CONTROLLER_UPDATED_EVENT)

        self.root.bind(CONTROLLER_UPDATED_EVENT, lambda e: self.update(self.message_queue.get()))
        t = threading.Thread(target=start_discoverer, args=(update_controllers_callback_threadsafe, self.quit_event))
        t.start()

        def on_quit():
            cemuhookserver.close_server()
            t_.join()

            self.quit_event.set()
            self.root.destroy()

        self.root.protocol("WM_DELETE_WINDOW", on_quit)

        self.root.mainloop()


if __name__ == "__main__":
    try:
        # This is a bit of a hack to check if bluetooth is available before starting the GUI
        # We create a dummy event loop and try to create a BleakScanner
        async def check_bluetooth():
            from bleak import BleakScanner
            try:
                async with BleakScanner():
                    pass
            except BleakBluetoothNotAvailableError as e:
                # Show a popup and exit
                root = tk.Tk()
                root.withdraw()
                tkinter.messagebox.showerror("Bluetooth Error", str(e))
                sys.exit(1)

        asyncio.run(check_bluetooth())

        window = ControllerWindow()
        window.init_interface()
        window.start()
    except BleakBluetoothNotAvailableError as e:
        # This should not happen, but just in case
        root = tk.Tk()
        root.withdraw()
        tkinter.messagebox.showerror("Bluetooth Error", str(e))
        sys.exit(1)
