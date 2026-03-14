import can
import time
import datetime
import multiprocessing as mp
import gvar_can
import re
import numpy as np
import copy
from typing import Dict
import os


import pygame
import pygame_gui
from pygame_gui import UIManager, PackageResource
from pygame_gui.elements import ui_horizontal_slider
from pygame_gui.elements import UI2DSlider
from pygame_gui.elements import UIButton
from pygame_gui.elements import UITextEntryLine
from pygame_gui.elements import UIDropDownMenu
from pygame_gui.elements import UILabel
from pygame_gui.elements.ui_text_box import UITextBox
from pygame_gui.windows import UIMessageWindow

import parameter_matrices as pmat
import ota

import pressure_calibration

COMPORT = 'COM31'


def write_parameters_to_actuator(bus, actuator_id_list):
    
    
    param_listener = Param_CAN_Listener()
    notifier = can.Notifier(bus, [param_listener])

    for actuator_id in actuator_id_list:
        # acquire the parameter matrices from parameter_matrices.py
        (param_inlet, param_outlet) = pmat.param_dict[actuator_id]
        # flatten the matrices to create a len=18 array for inlet and outlet
        inlet_params = param_inlet.flatten().tolist()
        outlet_params = param_outlet.flatten().tolist()
        # combine inlet and outlet params
        all_params = inlet_params + outlet_params

        param_listener.actuator_id = actuator_id
        # send the parameters to the actuator via CAN bus
        # first, write control bytes to 0b00000010 to enter parameter writing mode
        message = create_can_message(actuator_id, [0x00, 0x00, 0x00, 0x00, 0b00000010])
        bus.send(message)    

        # then, poll the can bus for acknowledgement from the actuator
        time.sleep(0.1)
        
        if param_listener.acknowledgement_received:
            # send the parameters one by one. the format: [param1_byte1, param1_byte2, param1_index_byte1, param1_index_byte2]
            for index, param in enumerate(all_params):
                param_listener.acknowledgement_received = False

                byte1 = param & 0x00FF
                byte2 = (param & 0xFF00) >> 8
                byte3 = index & 0x00FF
                byte4 = (index & 0xFF00) >> 8
                message = create_can_message(actuator_id, [byte1, byte2, byte3, byte4])
                bus.send(message)
                print(f"Actuator {hex(actuator_id)} sent parameter index {index}: value {param} as bytes [{byte1}, {byte2}]")
                
                # poll for acknowledgement before sending the next parameter
                while not param_listener.acknowledgement_received:
                    time.sleep(0.005)
        else:
            print("timeout waiting for acknowledgement for actuator ", hex(actuator_id))
    
    notifier.stop()
    

class Param_CAN_Listener(can.Listener):
    acknowledgement_received = False
    data_pointer = 0
    actuator_id = 0

    def __init__(self):
        super().__init__()
        self.data_pointer = 0
        actuator_id = 0

    def on_message_received(self, msg):
        if msg.arbitration_id == self.actuator_id:
            # if the received message is [0xFF, 0xFF, 0xFF, 0xFF]. then send parameter from start
            if msg.data[0] == 0xFF and msg.data[1] == 0xFF and msg.data[2] == 0xFF and msg.data[3] == 0xFF:
                self.acknowledgement_received = True
                self.data_pointer = 0
            elif msg.data[1] == 0xFF and msg.data[2] == 0xFF and msg.data[3] == 0xFF:
                self.acknowledgement_received = True
        pass


def create_canmsg_pressure_to_actuator(actuator_id, p, pdot_inlet, pdot_outlet, control_byte=0x00):
    # Convert the float pressure value to 4 bytes
    pressure_bytes = map_pressure_to_4bytes(p, pdot_inlet)
    # also, append the control bytes:
    # define control byte:
    # |0     |1            |2 |3 |4 |5 |6 |7 |
    # |on/off|test sequence|xx|xx|xx|xx|xx|xx|
    # xx: reserved for future use
    pressure_bytes.append(control_byte)    
    # Create a CAN message with the actuator ID and pressure bytes
    return create_can_message(actuator_id, pressure_bytes)
    
def map_pressure_to_4bytes(p, pdot):
    b1 = p & 0x000000FF
    b2 = (p & 0x0000FF00) >> 8
    b3 = (pdot & 0x00FF)
    b4 = (pdot & 0xFF00) >> 8
    return [b1, b2, b3, b4]

def map_pressure_from_4bytes(byte_list):
    v1_uint16_t = (byte_list[0] << 0) | (byte_list[1] << 8)
    v2_uint16_t = (byte_list[2] << 0) | (byte_list[3] << 8)
    return v1_uint16_t, v2_uint16_t

def map_data_from_6bytes(byte_list):
    v1_uint16_t = (byte_list[0] << 0) | (byte_list[1] << 8)  # pressure
    v2_uint16_t = (byte_list[2] << 0) | (byte_list[3] << 8)  # inlet PWM duty (0-1023)
    v3_uint16_t = (byte_list[4] << 0) | (byte_list[5] << 8)  # outlet PWM duty (0-1023)
    return v1_uint16_t, v2_uint16_t, v3_uint16_t

def create_can_message(arbitration_id, data, is_extended_id=False):
    try:
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id
        )
        return message
    except Exception as e:
        print(f"Error creating CAN message: {e}")
        return None


def prompt_user_for_pressure():
    """Prompt the user to input a pressure value."""
    while True:
        try:
            pressure = float(input("Enter pressure value (float): "))
            return pressure
        except ValueError:
            print("Invalid input. Please enter a valid float number.")

def mp_actuator_pressure_controller(send_parent_conn, mp_ctrl):
    try:
        while True:
            if mp_ctrl[gvar_can.mp_ctrl_start_sequence] == 1:
                
                mp_ctrl[gvar_can.mp_ctrl_start_sequence] = 0
                print("Starting pressure sequence...")
                
                # # step response test
                # mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
                # send_parent_conn.send([1000, 0xaaaa])
                # time.sleep(3)
                # send_parent_conn.send([3200, 0xaaaa])
                # time.sleep(3)
                # send_parent_conn.send([1000, 0xaaaa])
                # time.sleep(3)
                # mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1

                # # sine wave test
                freq = 2 # Hz
                amplitude = 800 # pressure units
                offset = 2500 # pressure units
                duration = 6 # seconds
                send_parent_conn.send([offset, 0xFFFF])
                time.sleep(3)

                mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
                start_time = time.time()
                while time.time() - start_time < duration:
                    t = time.time() - start_time
                    pressure = int(offset + amplitude * np.sin(2 * np.pi * freq * t))
                    send_parent_conn.send([pressure, 0xC8C8])
                    # time.sleep(0.005) # 200 Hz update rate

                send_parent_conn.send([offset, 0xFFFF])
            
                time.sleep(0.1)
                mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
                send_parent_conn.send([1000, 0xFFFF])
                print("Pressure sequence completed.")

                
            else:
                time.sleep(0.1)

    except Exception as e:
        print(f"Error in actuator pressure controller: {e}")




def mp_data_recoder(mp_actuator_t_p_pdot, mp_ctrl):
    try:
        data_matrix = []

        while True:
            if mp_ctrl[gvar_can.mp_ctrl_start_recording] == 1 and mp_ctrl[gvar_can.mp_ctrl_stop_recording] == 0:
                if not mp_actuator_t_p_pdot.empty():
                    data = mp_actuator_t_p_pdot.get()

                    timestamp = data[0]
                    actuator_id = data[1]
                    actual_pressure = data[2]
                    inlet_pwm = data[3]
                    outlet_pwm = data[4]
                    target_pressure = data[5]

                    date_time_vector = np.array([
                        timestamp.year,
                        timestamp.month,
                        timestamp.day,
                        timestamp.hour,
                        timestamp.minute,
                        timestamp.second,
                        timestamp.microsecond
                    ])

                    final_vector = np.concatenate((date_time_vector, np.array([actuator_id, actual_pressure, inlet_pwm, outlet_pwm, target_pressure])))

                    # print(final_vector)
                    data_matrix.append(final_vector)


            elif mp_ctrl[gvar_can.mp_ctrl_stop_recording] == 1:
                
                data_matrix = np.array(data_matrix)

                # Define the directory and file path
                directory = "./datalog"

                filename = "data_log_" + str(datetime.datetime.now())  # .npy is a common extension for numpy arrays
                filename = filename.replace(" ", "_")
                filename = filename.replace(":", "-")
                filename = filename.replace(".", "-")
                filename += ".npy"

                filepath = os.path.join(directory, filename)
                # Create the directory if it doesn't exist
                os.makedirs(directory, exist_ok=True)
                # Save the array to the file

                np.save(filepath, data_matrix)

                mp_ctrl[gvar_can.mp_ctrl_start_recording] = 0
                mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 0
                data_matrix = []

                print(f"Data saved to {filepath}")

            else:
                time.sleep(0.01)
    except Exception as e:
        print(f"Error in data recorder: {e}")
    pass    



# Define a custom listener class
class VEMA_CAN_Listener(can.Listener):
    # current_msg = None
    # new_msg_flag = False
    
    # currently, there will be 24 actuators, where the id range from 0x101 to 0x124
    actuator_ids = [i for i in range(0x101, 0x125)]
    # the listener will store the latest pressure and PWM duty for each actuator
    actuator_pressures : Dict[int, int] = {aid: 0 for aid in actuator_ids}
    actuator_pressure_dots : Dict[int, int] = {aid: 0 for aid in actuator_ids}
    actuator_pwm_duty_inlet : Dict[int, int] = {aid: 0 for aid in actuator_ids}
    actuator_pwm_duty_outlet: Dict[int, int] = {aid: 0 for aid in actuator_ids}

    def on_message_received(self, msg):
        # self.current_msg = copy.deepcopy(msg)
        # self.new_msg_flag = True

        if msg.arbitration_id in self.actuator_ids:
            # seperate old and new format by checking the length of the data.
            if len(msg.data) == 4:
                v1_recv, v2_recv = map_pressure_from_4bytes(msg.data)
                self.actuator_pressures[msg.arbitration_id] = v1_recv
                self.actuator_pressure_dots[msg.arbitration_id] = v2_recv
            elif len(msg.data) == 6:
                v1_recv, v2_recv, v3_recv = map_data_from_6bytes(msg.data)
                self.actuator_pressures[msg.arbitration_id] = v1_recv
                self.actuator_pwm_duty_inlet[msg.arbitration_id] = v2_recv
                self.actuator_pwm_duty_outlet[msg.arbitration_id] = v3_recv


    def on_error(self, exc: Exception) -> bool:
        """Callback for error handling."""
        print(f"CAN error occurred: {exc}")
        return True # Return True to indicate the error was handled
    


def subroutine_CAN_handler(send_child_conn, mp_actuator_status ,mp_actuator_t_p_pdot, mp_ctrl):
    try:
        bus = None
        # try connection until success
        while bus is None:
            try:
                bus = can.interface.Bus(bustype='slcan', channel=COMPORT, bitrate=1000000)
            except Exception as e:
                print(f"Failed to connect to CAN bus: {e}. Retrying in 2 seconds...")
                time.sleep(2)

        print("Starting CAN bus...")

        
        # set up a listener, which will automatically go to the callback function once a message is received
        # inside the call back function, we will update the actuator pressure and pressure_dot values
        vema_listener = VEMA_CAN_Listener()
        notifier = can.Notifier(bus, [vema_listener])
        print("Notifier started. Waiting for CAN messages...")


        p_sent = 0
        p_dot_sent = 0
        p_value = 0
        pdot_value = 0

        print_time = time.time()
        log_time   = time.time()

        while True:
            # poll the send_child_conn to see if there is new target pressure and pressure_dot values
            # if there is, update the p_value and pdot_value for this actuator
            if send_child_conn.poll():
                # this connection will receive all actuator's target pressure, target pressure_dot values, and control bytes.
                # the format is: [[target_p_val, target_pdot_val, control_byte, target_id], ...]
                # this is a 2d array, each row is for one actuator.
                incoming_msg = send_child_conn.recv()

                if isinstance(incoming_msg, dict) and incoming_msg.get("type") == "ota_start":
                    device_id = int(incoming_msg.get("device_id", 0))
                    firmware_bin = incoming_msg.get("firmware", "Valve_not_embedded_XL.bin")
                    send_child_conn.send({
                        "type": "ota_status",
                        "status": "started",
                        "message": f"OTA started for {hex(device_id)}"
                    })

                    try:
                        notifier.stop()
                        ota_success = ota.perform_can_ota_on_bus(bus, device_id, firmware_bin)
                        if ota_success:
                            send_child_conn.send({
                                "type": "ota_status",
                                "status": "finished",
                                "message": f"OTA finished for {hex(device_id)}"
                            })
                        else:
                            send_child_conn.send({
                                "type": "ota_status",
                                "status": "failed",
                                "message": f"OTA failed for {hex(device_id)}"
                            })
                    except Exception as e:
                        send_child_conn.send({
                            "type": "ota_status",
                            "status": "failed",
                            "message": f"OTA exception for {hex(device_id)}: {e}"
                        })
                    finally:
                        notifier = can.Notifier(bus, [vema_listener])

                    continue

                if not isinstance(incoming_msg, list):
                    print(f"Unknown command payload: {incoming_msg}")
                    continue

                actuator_target_package = incoming_msg
                
                for each_actuator_target in actuator_target_package:
                    # print(each_actuator_target)
                    target_p_val, target_pdot_val, control_byte, target_id = each_actuator_target

                    # acquire p_val and pwm duty from the vema listener for this target_id
                    p_val = vema_listener.actuator_pressures.get(target_id, 0)
                    pwm_duty_inlet = vema_listener.actuator_pwm_duty_inlet.get(target_id, 0)
                    pwm_duty_outlet = vema_listener.actuator_pwm_duty_outlet.get(target_id, 0)
                    # update the actuator status control byte
                    # it is important to replace the array, not update the value inside the array, because the mp_actuator_status is a multiprocessing dict
                    # updating the value inside the array may not be recognized by other processes
                    mp_actuator_status[target_id] = [p_val, pwm_duty_inlet, pwm_duty_outlet, target_p_val, target_pdot_val, control_byte]
            
            # # if no command is received, still update the actual pressure and pressure_dot values from the listener
            # else:
            #     for aid in gvar_can.actuator_id_list:
            #         # acquire p_val and pdot_val from the vema listener for this target_id
            #         p_val = vema_listener.actuator_pressures.get(aid, 0)
            #         pdot_val = vema_listener.actuator_pressure_dots.get(aid, 0)
            #         # use the old target pressure and pressure_dot values
            #         target_p_val = mp_actuator_status[aid][2]
            #         target_pdot_val = mp_actuator_status[aid][3]
            #         control_byte = mp_actuator_status[aid][4]
            #         # update the actuator status control byte
            #         # it is important to replace the array, not update the value inside the array, because the mp_actuator_status is a multiprocessing dict
            #         # updating the value inside the array may not be recognized by other processes
            #         mp_actuator_status[aid] = [p_val, pdot_val, target_p_val, target_pdot_val, control_byte]

            # send pressure commands to all actuators.
            for aid in gvar_can.actuator_id_list:
                # update the actual pressure and PWM duty values from the listener
                p_val = vema_listener.actuator_pressures.get(aid, 0)
                pwm_duty_inlet = vema_listener.actuator_pwm_duty_inlet.get(aid, 0)
                pwm_duty_outlet = vema_listener.actuator_pwm_duty_outlet.get(aid, 0)
                # use the old target pressure and pressure_dot values
                target_p_val = mp_actuator_status[aid][3]
                target_pdot_val = mp_actuator_status[aid][4]
                control_byte = mp_actuator_status[aid][5]
                # update the actuator status control byte
                # it is important to replace the array, not update the value inside the array, because the mp_actuator_status is a multiprocessing dict
                # updating the value inside the array may not be recognized by other processes
                mp_actuator_status[aid] = [p_val, pwm_duty_inlet, pwm_duty_outlet, target_p_val, target_pdot_val, control_byte]


                # for each actuator, set the target pressure and pressure_dot values
                # grab those target values from the shared dict mp_actuator_status
                p_value = int(target_p_val)
                pdot_value = int(target_pdot_val)
                actuator_ctrl = int(control_byte)
                canmsg = create_canmsg_pressure_to_actuator(actuator_id=aid, p=p_value, pdot_inlet=pdot_value, pdot_outlet=0xFFFF, control_byte=actuator_ctrl)
                bus.send(canmsg)
                # time.sleep(0.001) # small delay to avoid flooding the bus

            # logging data for all actuators
            if mp_ctrl[gvar_can.mp_ctrl_start_recording] == 1:
                if time.time() - log_time > 0.0025:  # 400hz
                    # log data for all actuators
                    timestamp = datetime.datetime.now()
                    for aid in gvar_can.actuator_id_list:
                        actual_pressure = vema_listener.actuator_pressures.get(aid, 0)
                        inlet_pwm = vema_listener.actuator_pwm_duty_inlet.get(aid, 0)
                        outlet_pwm = vema_listener.actuator_pwm_duty_outlet.get(aid, 0)
                        target_p = mp_actuator_status[aid][3]
                        # put [timestamp, actuator_id, actual_pressure, inlet_pwm, outlet_pwm, target_pressure] into queue
                        mp_actuator_t_p_pdot.put([timestamp, aid, actual_pressure, inlet_pwm, outlet_pwm, target_p])
                    log_time = time.time()

            printing = False
            if printing:
                # print status
                if time.time() - print_time > 0.1:
                    # print with the format:
                    # ||aid0:p_val||aid1:p_val||.....
                    # when printing, p_val should always be 4 digits, pad with leading spaces if necessary
                    status_str = "||"
                    for aid in gvar_can.actuator_id_list:
                        status_str += f"{hex(aid)}:{mp_actuator_status[aid][0]:4}||"
                    
                    # print(status_str)

                    print_time = time.time()

    except Exception as e:
        print(f"Error in CAN handler: {e}")


def console(send_parent_conn, mp_actuator_status, mp_ctrl):
    quit_all = False
    try:
        while not quit_all:
            try:
                inchars = input('Enter command or (\'h\' for list of commands)\n')
                if len(inchars)>0:
                    c1 = inchars[0].lower()
                    args = re.findall(r'[-+]?\d+\.?\d*', inchars)
                    args = [float(num) if '.' in num else int(num) for num in args]
                    # shutdown
                    if c1 == 'q':
                        quit_all = True
                        pass
                    if c1 == 'c': # calibration
                        print("================== Starting pressure calibration ==================")
                        pressure_calibration.pressure_calibration(send_parent_conn, mp_actuator_status)
                        print("================== Pressure calibration completed ==================")

                    elif c1 == 'b':
                        # arg[0] is the target pressure, arg[1] is the target id
                        # first, generate a 2d array to hold the target pressure and target pressure_dot for each actuator
                        actuator_command_package = []
                        for target_id in gvar_can.actuator_id_list:
                            target_pressure = int(mp_actuator_status[target_id][2])
                            target_pdot = int(mp_actuator_status[target_id][3])
                            control_byte = int(mp_actuator_status[target_id][4])

                            if args[1] == target_id or args[1] == gvar_can.BROADCAST_CANID:
                                target_pressure = int(args[0])
                                control_byte    = 0x01
                                target_pdot     = 0xFFFF
                                
                            actuator_command_package.append([target_pressure, target_pdot, control_byte, target_id])

                        send_parent_conn.send(actuator_command_package)

                        print(f"Sent target pressure {args[0]} to actuator ID {hex(args[1])}")
                        
                    elif c1 == 'd':
                        pdot1 = int(args[0])
                        pdot2 = int(args[1])
                        target_id = int(args[2])
                        print(pdot1, pdot2)
                        target_pdot = (pdot1 & 0x00FF) | ((pdot2 << 8) & 0xFF00)
                        control_byte = 0x01
                        send_parent_conn.send([target_pressure, target_pdot, control_byte, target_id])

                    elif c1 == 'r':
                        if args[0] == 1:
                            print("start recording")
                            mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
                            
                        elif args[0] == 0:
                            print("stop recording")
                            mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
                    
                    elif c1 == 'a':
                        if args[0] == 1:
                            print("start sequence")
                            mp_ctrl[gvar_can.mp_ctrl_start_sequence] = 1

                    elif c1 == 's':
                        print("stop all")
                        actuator_command_package = []
                        for target_id in gvar_can.actuator_id_list:
                            target_pressure = int(mp_actuator_status[target_id][2])
                            target_pdot = int(mp_actuator_status[target_id][3])
                            control_byte = int(mp_actuator_status[target_id][4])

                            if args[0] == target_id or args[0] == gvar_can.BROADCAST_CANID:
                                target_pressure = 0x00
                                control_byte    = 0x00
                                target_pdot     = 0xFFFF
                                
                            actuator_command_package.append([target_pressure, target_pdot, control_byte, target_id])

                        send_parent_conn.send(actuator_command_package)

                        print(f"Sent target pressure {args[0]} to actuator ID {hex(args[1])}")
                        pass
                    elif c1 == 'p':
                        print("here")
                        pass
                pass
            except Exception as e:
                print("invalid command, ", e)
    except Exception as e:
        print("console: ", e)


# a GUI using pygame_gui only have 24 sliders to control the target pressure of each actuator
# this gui is a standalone process that sends the target pressure values to the main process via a multiprocessing pipe
# this is the definition of the pygame gui class
class CANPygameGUI:
    def __init__(self, send_conn_parent, mp_actuator_status, mp_ctrl):
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100, 45)
        pygame.init()
        pygame.display.set_caption("CAN Bus Actuator Control")
        
        self.send_conn_parent = send_conn_parent
        self.mp_actuator_status = mp_actuator_status
        self.mp_ctrl = mp_ctrl
        
        self.resolution = (1600, 800)
        self.window_surface = pygame.display.set_mode(self.resolution)
        self.background_surface = None
        
        self.ui_manager = UIManager(self.resolution,
                                    PackageResource(package='data.themes',
                                                    resource='theme_2.json'))
        self.ui_manager.preload_fonts([{'name': 'fira_code', 'point_size': 10, 'style': 'bold'},
                                       {'name': 'fira_code', 'point_size': 10, 'style': 'regular'},
                                       {'name': 'fira_code', 'point_size': 10, 'style': 'italic'},
                                       {'name': 'fira_code', 'point_size': 12, 'style': 'bold'}])
        
        # UI elements
        self.actuator_sliders = {}  # Dictionary to hold sliders for each actuator
        self.actuator_labels = {}   # Dictionary to hold labels for each actuator
        self.actuator_value_labels = {}  # Dictionary to hold current value labels
        self.actuator_test_buttons = {}  # Dictionary to hold stop/start toggle buttons for each actuator
        self.actuator_running_state = {}  # Dictionary to track running state for each actuator (True=running, False=stopped)
        
        self.toggle_recording_button = None
        self.recording_state = False  # False=not recording, True=recording
        self.start_sequence_button = None
        self.stop_all_button = None
        self.broadcast_button = None
        
        self.status_textbox = None
        self.broadcast_pressure_entry = None
        self.ota_device_id_entry = None
        self.start_ota_button = None
        self.ota_firmware_bin = "Valve_not_embedded_XL.bin"
        self.ota_running = False
        self._last_ota_running = False
        
        self.recreate_ui()
        
        self.clock = pygame.time.Clock()
        self.running = True
        
    def recreate_ui(self):
        self.ui_manager.clear_and_reset()
        
        self.background_surface = pygame.Surface(self.resolution)
        self.background_surface.fill(self.ui_manager.get_theme().get_colour('dark_bg'))
        
        # Create sliders for all 24 actuators
        slider_width = 150
        slider_height = 20
        label_width = 80
        label_height = 20
        value_label_width = 280
        
        start_x = 10
        start_y = 50
        y_spacing = 30
        
        # Create 24 sliders in 2 columns
        for i, actuator_id in enumerate(gvar_can.actuator_id_list):
            column = i // 12  # 12 actuators per column
            row = i % 12
            
            x = start_x + column * 590
            y = start_y + row * y_spacing
            
            # Create label for actuator ID
            label = UILabel(
                relative_rect=pygame.Rect((x, y), (label_width, label_height)),
                text=f"{hex(actuator_id)}:",
                manager=self.ui_manager
            )
            self.actuator_labels[actuator_id] = label
            
            # Create slider for target pressure (range 0-4095)
            slider = ui_horizontal_slider.UIHorizontalSlider(
                pygame.Rect((x + label_width + 5, y), (slider_width, slider_height)),
                start_value=1000,
                value_range=(0, 4095),
                manager=self.ui_manager
            )
            self.actuator_sliders[actuator_id] = slider
            
            # Create label to display current value
            value_label = UILabel(
                relative_rect=pygame.Rect((x + label_width + slider_width + 10, y), (value_label_width, label_height)),
                text=f"T:1000 A:0 in:0 out:0",
                manager=self.ui_manager
            )
            self.actuator_value_labels[actuator_id] = value_label
            
            # Create stop/start toggle button beside the slider
            stop_button = UIButton(
                pygame.Rect((x + label_width + slider_width + value_label_width + 20, y), (60, label_height)),
                'Stopped',
                self.ui_manager
            )
            # Set initial color to red for stopped state
            stop_button.colours['normal_bg'] = pygame.Color('#AA0000')
            stop_button.colours['hovered_bg'] = pygame.Color('#CC0000')
            stop_button.colours['active_bg'] = pygame.Color('#880000')
            stop_button.rebuild()
            self.actuator_test_buttons[actuator_id] = stop_button
            self.actuator_running_state[actuator_id] = False  # Initialize as stopped
        
        # Create control buttons on the right side
        button_x = start_x + 1240
        button_y = start_y
        button_width = 180
        button_height = 40
        button_spacing = 50
        
        self.toggle_recording_button = UIButton(
            pygame.Rect((button_x, button_y), (button_width, button_height)),
            'Start Recording',
            self.ui_manager
        )
        # Set initial color to green for not recording state
        self.toggle_recording_button.colours['normal_bg'] = pygame.Color('#00AA00')
        self.toggle_recording_button.colours['hovered_bg'] = pygame.Color('#00CC00')
        self.toggle_recording_button.colours['active_bg'] = pygame.Color('#008800')
        self.toggle_recording_button.rebuild()
        
        self.start_sequence_button = UIButton(
            pygame.Rect((button_x, button_y + button_spacing), (button_width, button_height)),
            'Start Sequence',
            self.ui_manager
        )
        
        self.stop_all_button = UIButton(
            pygame.Rect((button_x, button_y + button_spacing * 2), (button_width, button_height)),
            'Stop All',
            self.ui_manager
        )
        
        # Broadcast control
        UILabel(
            relative_rect=pygame.Rect((button_x, button_y + button_spacing * 3), (button_width, 25)),
            text="Broadcast Pressure:",
            manager=self.ui_manager
        )
        
        self.broadcast_pressure_entry = UITextEntryLine(
            relative_rect=pygame.Rect((button_x, button_y + button_spacing * 3 + 30), (button_width, 30)),
            manager=self.ui_manager
        )
        self.broadcast_pressure_entry.set_text("1000")
        
        self.broadcast_button = UIButton(
            pygame.Rect((button_x, button_y + button_spacing * 3 + 70), (button_width, button_height)),
            'Broadcast to All',
            self.ui_manager
        )

        broadcast_button_bottom = button_y + button_spacing * 3 + 70 + button_height
        ota_top = broadcast_button_bottom + 20
        status_top = ota_top + 120

        # OTA control
        UILabel(
            relative_rect=pygame.Rect((button_x, ota_top), (button_width, 25)),
            text="OTA Device ID (hex/dec):",
            manager=self.ui_manager
        )

        self.ota_device_id_entry = UITextEntryLine(
            relative_rect=pygame.Rect((button_x, ota_top + 30), (button_width, 30)),
            manager=self.ui_manager
        )
        self.ota_device_id_entry.set_text("0x102")

        self.start_ota_button = UIButton(
            pygame.Rect((button_x, ota_top + 70), (button_width, button_height)),
            'Start OTA',
            self.ui_manager
        )
        self.start_ota_button.colours['normal_bg'] = pygame.Color('#0055AA')
        self.start_ota_button.colours['hovered_bg'] = pygame.Color('#0066CC')
        self.start_ota_button.colours['active_bg'] = pygame.Color('#004488')
        self.start_ota_button.rebuild()
        
        # Status text box
        self.status_textbox = UITextBox(
            html_text="<font color=#00FF00>CAN Bus Status: Ready</font><br>",
            relative_rect=pygame.Rect((button_x, status_top), 
                                     (button_width, 200)),
            manager=self.ui_manager
        )
        
        # Title
        UILabel(
            relative_rect=pygame.Rect((10, 10), (400, 30)),
            text="CAN Bus Actuator Control - 24 Actuators",
            manager=self.ui_manager
        )
        
    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                
            self.ui_manager.process_events(event)
            
            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == self.toggle_recording_button:
                    # Toggle recording state
                    self.recording_state = not self.recording_state
                    
                    if self.recording_state:
                        # Start recording
                        print("Start recording")
                        self.mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
                        self.mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 0
                        self.append_status("Recording started")
                        
                        # Update button to red "Stop Recording"
                        self.toggle_recording_button.set_text('Stop Recording')
                        self.toggle_recording_button.colours['normal_bg'] = pygame.Color('#AA0000')
                        self.toggle_recording_button.colours['hovered_bg'] = pygame.Color('#CC0000')
                        self.toggle_recording_button.colours['active_bg'] = pygame.Color('#880000')
                        self.toggle_recording_button.rebuild()
                    else:
                        # Stop recording
                        print("Stop recording")
                        self.mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
                        self.append_status("Recording stopped")
                        
                        # Update button to green "Start Recording"
                        self.toggle_recording_button.set_text('Start Recording')
                        self.toggle_recording_button.colours['normal_bg'] = pygame.Color('#00AA00')
                        self.toggle_recording_button.colours['hovered_bg'] = pygame.Color('#00CC00')
                        self.toggle_recording_button.colours['active_bg'] = pygame.Color('#008800')
                        self.toggle_recording_button.rebuild()
                    
                elif event.ui_element == self.start_sequence_button:
                    print("Start sequence")
                    self.mp_ctrl[gvar_can.mp_ctrl_start_sequence] = 1
                    self.append_status("Sequence started")
                    
                elif event.ui_element == self.stop_all_button:
                    print("Stop all actuators")
                    actuator_command_package = []
                    for target_id in gvar_can.actuator_id_list:
                        current_status = self.mp_actuator_status[target_id]
                        target_pressure = int(current_status[3])
                        target_pdot     = int(current_status[4])
                        control_byte    = 0x00
                            
                        actuator_command_package.append([target_pressure, target_pdot, control_byte, target_id])
                        
                        # Update individual button state to stopped (red)
                        self.actuator_running_state[target_id] = False
                        toggle_button = self.actuator_test_buttons[target_id]
                        toggle_button.set_text('Stopped')
                        toggle_button.colours['normal_bg'] = pygame.Color('#AA0000')
                        toggle_button.colours['hovered_bg'] = pygame.Color('#CC0000')
                        toggle_button.colours['active_bg'] = pygame.Color('#880000')
                        toggle_button.rebuild()

                    self.send_conn_parent.send(actuator_command_package)
                    self.append_status("All actuators stopped")
                    
                elif event.ui_element == self.broadcast_button:
                    try:
                        broadcast_pressure = int(self.broadcast_pressure_entry.get_text())
                        self.broadcast_pressure_to_all(broadcast_pressure)
                        
                        # Update all sliders, displays, and button states
                        for actuator_id in gvar_can.actuator_id_list:
                            # Update slider value
                            self.actuator_sliders[actuator_id].set_current_value(broadcast_pressure)
                            
                            # Update display label
                            status = self.mp_actuator_status[actuator_id]
                            actual_pressure = status[0]
                            duty_in = status[1]
                            duty_out = status[2]
                            self.actuator_value_labels[actuator_id].set_text(f"T:{broadcast_pressure} A:{actual_pressure} in:{duty_in} out:{duty_out}")
                            
                            # Update button state to running (green)
                            self.actuator_running_state[actuator_id] = True
                            toggle_button = self.actuator_test_buttons[actuator_id]
                            toggle_button.set_text('Running')
                            toggle_button.colours['normal_bg'] = pygame.Color('#00AA00')
                            toggle_button.colours['hovered_bg'] = pygame.Color('#00CC00')
                            toggle_button.colours['active_bg'] = pygame.Color('#008800')
                            toggle_button.rebuild()
                        
                        self.append_status(f"Broadcast pressure {broadcast_pressure} to all actuators")
                    except ValueError:
                        self.append_status("Invalid pressure value")

                elif event.ui_element == self.start_ota_button:
                    if self.ota_running:
                        self.append_status("OTA is already running")
                    else:
                        try:
                            device_id = self.parse_device_id(self.ota_device_id_entry.get_text())
                            self.start_ota(device_id)
                        except ValueError:
                            self.append_status("Invalid OTA device ID. Use hex like 0x102 or decimal")
                
                # Handle stop/start toggle button presses
                else:
                    for actuator_id, toggle_button in self.actuator_test_buttons.items():
                        if event.ui_element == toggle_button:
                            # Toggle the running state
                            is_running = self.actuator_running_state[actuator_id]
                            
                            if is_running:
                                # Currently running, so stop it
                                self.send_stop_to_actuator(actuator_id)
                                self.actuator_running_state[actuator_id] = False
                                # Update button to red "Stopped"
                                toggle_button.set_text('Stopped')
                                toggle_button.colours['normal_bg'] = pygame.Color('#AA0000')
                                toggle_button.colours['hovered_bg'] = pygame.Color('#CC0000')
                                toggle_button.colours['active_bg'] = pygame.Color('#880000')
                                toggle_button.rebuild()
                                self.append_status(f"Actuator {hex(actuator_id)} stopped")
                            else:
                                # Currently stopped, so start it
                                self.send_start_to_actuator(actuator_id)
                                self.actuator_running_state[actuator_id] = True
                                # Update button to green "Running"
                                toggle_button.set_text('Running')
                                toggle_button.colours['normal_bg'] = pygame.Color('#00AA00')
                                toggle_button.colours['hovered_bg'] = pygame.Color('#00CC00')
                                toggle_button.colours['active_bg'] = pygame.Color('#008800')
                                toggle_button.rebuild()
                                self.append_status(f"Actuator {hex(actuator_id)} running")
                            break
                        
            if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                # Check which slider was moved and update the corresponding actuator
                for actuator_id, slider in self.actuator_sliders.items():
                    if event.ui_element == slider:
                        target_pressure = int(slider.get_current_value())
                        # only send pressure if the actuator is running
                        if self.actuator_running_state[actuator_id]:
                            self.send_pressure_to_actuator(actuator_id, target_pressure)
                        # Immediately update the display label to sync with slider
                        status = self.mp_actuator_status[actuator_id]
                        actual_pressure = status[0]
                        duty_in = status[1]
                        duty_out = status[2]
                        self.actuator_value_labels[actuator_id].set_text(f"T:{target_pressure} A:{actual_pressure} in:{duty_in} out:{duty_out} psi:{pmat.adc_to_psi(actuator_id=actuator_id, adc_value=actual_pressure):.1f}")

    @staticmethod
    def parse_device_id(text):
        value = int(text.strip(), 0)
        if value < 0 or value > 0x7FF:
            raise ValueError("CAN standard ID must be in [0, 0x7FF]")
        return value

    def start_ota(self, device_id):
        if not os.path.exists(self.ota_firmware_bin):
            self.append_status(f"OTA firmware not found: {self.ota_firmware_bin}")
            return

        self.ota_running = True
        self.update_ota_button_state()
        self.send_conn_parent.send({
            "type": "ota_start",
            "device_id": int(device_id),
            "firmware": self.ota_firmware_bin
        })
        self.append_status(f"OTA requested for {hex(device_id)}")

    def process_incoming_messages(self):
        while self.send_conn_parent.poll():
            incoming_msg = self.send_conn_parent.recv()
            if isinstance(incoming_msg, dict) and incoming_msg.get("type") == "ota_status":
                status = incoming_msg.get("status", "")
                message = incoming_msg.get("message", "")

                if message:
                    self.append_status(message)

                if status in ["finished", "failed"]:
                    self.ota_running = False
                    self.update_ota_button_state()

    def update_ota_button_state(self):
        if self._last_ota_running == self.ota_running:
            return

        if self.ota_running:
            self.start_ota_button.set_text('OTA Running...')
            self.start_ota_button.colours['normal_bg'] = pygame.Color('#555555')
            self.start_ota_button.colours['hovered_bg'] = pygame.Color('#666666')
            self.start_ota_button.colours['active_bg'] = pygame.Color('#444444')
        else:
            self.start_ota_button.set_text('Start OTA')
            self.start_ota_button.colours['normal_bg'] = pygame.Color('#0055AA')
            self.start_ota_button.colours['hovered_bg'] = pygame.Color('#0066CC')
            self.start_ota_button.colours['active_bg'] = pygame.Color('#004488')

        self.start_ota_button.rebuild()
        self._last_ota_running = self.ota_running
                        
    def send_pressure_to_actuator(self, target_id, target_pressure):
        """Send target pressure to a specific actuator"""
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            current_status = self.mp_actuator_status[aid]
            if aid == target_id:
                # Update this actuator
                actuator_command_package.append([target_pressure, 0xFFFF, 0x01, aid])
            else:
                # Keep current values for other actuators
                actuator_command_package.append([int(current_status[3]), int(current_status[4]), int(current_status[5]), aid])
        
        self.send_conn_parent.send(actuator_command_package)
    
    def send_start_to_actuator(self, target_id):
        """Send start command (control_byte = 0x01) to a specific actuator"""
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            current_status = self.mp_actuator_status[aid]
            if aid == target_id:
                # Get the target pressure from the slider
                target_pressure = int(self.actuator_sliders[target_id].get_current_value())
                # Send start command with control_byte = 0x01
                actuator_command_package.append([target_pressure, 0xFFFF, 0x01, aid])
            else:
                # Keep current values for other actuators
                actuator_command_package.append([int(current_status[3]), int(current_status[4]), int(current_status[5]), aid])
        
        self.send_conn_parent.send(actuator_command_package)
    
    def send_stop_to_actuator(self, target_id):
        """Send stop command (control_byte = 0x00, pressure = 0) to a specific actuator"""
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            current_status = self.mp_actuator_status[aid]
            if aid == target_id:
                # Send stop command with control_byte = 0x00 and pressure = 0
                actuator_command_package.append([0x00, 0xFFFF, 0x00, aid])
            else:
                # Keep current values for other actuators
                actuator_command_package.append([int(current_status[3]), int(current_status[4]), int(current_status[5]), aid])
        
        self.send_conn_parent.send(actuator_command_package)
        
    def broadcast_pressure_to_all(self, target_pressure):
        """Send target pressure to all actuators"""
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            actuator_command_package.append([target_pressure, 0xFFFF, 0x01, aid])
        
        self.send_conn_parent.send(actuator_command_package)
        
    def append_status(self, message):
        """Append a message to the status textbox"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        current_text = self.status_textbox.html_text
        new_text = f"{current_text}<font color=#FFFFFF>[{timestamp}]</font> {message}<br>"
        self.status_textbox.html_text = new_text
        self.status_textbox.rebuild()
        
    def update_actuator_displays(self):
        """Update the display labels with current actuator values"""
        for actuator_id in gvar_can.actuator_id_list:
            status = self.mp_actuator_status[actuator_id]
            actual_pressure = status[0]
            target_pressure = self.actuator_sliders[actuator_id].get_current_value()
            
            # Update the value label
            duty_in = status[1]
            duty_out = status[2]
            psi_value = pmat.adc_to_psi(actuator_id=actuator_id, adc_value=actual_pressure)
            self.actuator_value_labels[actuator_id].set_text(f"T:{target_pressure} A:{actual_pressure} in:{duty_in} out:{duty_out} psi:{psi_value:.1f}")
            
    def run(self):
        while self.running:
            time_delta = self.clock.tick(60) / 1000.0  # 60 FPS
            
            self.process_events()
            self.process_incoming_messages()
            self.update_ota_button_state()
            self.ui_manager.update(time_delta)
            
            # Update actuator displays
            self.update_actuator_displays()
            
            # Draw
            self.window_surface.blit(self.background_surface, (0, 0))
            self.ui_manager.draw_ui(self.window_surface)
            pygame.display.update()


# this is the entry point for the pygame gui process
def can_pygame_gui(send_conn_parent, mp_actuator_status, mp_ctrl):
    try:
        app = CANPygameGUI(send_conn_parent, mp_actuator_status, mp_ctrl)
        app.run()
    except Exception as e:
        print(f"Error in CAN pygame GUI: {e}")
    finally:
        pygame.quit()



if __name__ == "__main__":
    bus = None

    # Initialize the multiprocessing manager and CAN bus in gvar_can
    gvar_can.gvar_manager = mp.Manager()
    # Initialize the CAN bus.
    
    # Shared list to store actuator pressures
    mp_actuator_t_p_pdot = gvar_can.gvar_manager.Queue()
    mp_ctrl = gvar_can.gvar_manager.list()
    # actuator status: key = actuator can bus id, data = list [actual_pressure, pwm_duty_inlet, pwm_duty_outlet, target_pressure, target_pdot, control_byte]
    mp_actuator_status = gvar_can.gvar_manager.dict()

    for i in gvar_can.actuator_id_list:
        mp_actuator_status[i] = [0, 0, 0, 0, 0, 0b00000000] # pressure, pwm_duty_inlet, pwm_duty_outlet, target_pressure, target_pdot, control_byte
    for i in range(50):
        mp_ctrl.append(0)

    send_parent_conn, send_child_conn = mp.Pipe()

    print("Connecting to CAN bus...")
    while bus is None:
        try:
            bus = can.interface.Bus(bustype='slcan', channel=COMPORT, bitrate=1000000)
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}. Retrying in 2 seconds...")
            time.sleep(2)

    # Receiving CAN messages
    try:
        # first, write the parameters to all actuators
        print("Writing parameters to all actuators...")
        # write_parameters_to_actuator(bus, [0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107, 0x108])
        # write_parameters_to_actuator(bus, [0x106])
        # close this bus instance
        bus.shutdown()
        time.sleep(0.1)
        # create the process for CAN handler
        p_can_handler = mp.Process(target=subroutine_CAN_handler, args=(send_child_conn, mp_actuator_status, mp_actuator_t_p_pdot, mp_ctrl))
        # create another process for the data recoder
        p_data_recoder = mp.Process(target=mp_data_recoder, args=(mp_actuator_t_p_pdot, mp_ctrl))
        # create another process that takes in the send_parent_conn and send the target pressure and target pressure_dot to the CAN handler
        p_actuator_pressure_controller = mp.Process(target=mp_actuator_pressure_controller, args=(send_parent_conn, mp_ctrl))
        # create another process for the pygame gui

        gui = True
        if gui:
            p_can_pygame_gui = mp.Process(target=can_pygame_gui, args=(send_parent_conn, mp_actuator_status, mp_ctrl))
        
        # start all processes
        p_data_recoder.start()
        p_can_handler.start()
        p_actuator_pressure_controller.start()
        if gui:
            p_can_pygame_gui.start()

        # this function blocks until user quits
        # this is the main console function with terminal being the user interface
        console(send_parent_conn, mp_actuator_status, mp_ctrl)  # Run the main console function


        # After user quits, terminate all processes
        p_data_recoder.terminate()
        p_can_handler.terminate()
        p_actuator_pressure_controller.terminate()
        if gui:
            p_can_pygame_gui.terminate()

        p_data_recoder.join()
        p_can_handler.join()
        p_actuator_pressure_controller.join()
        if gui:
            p_can_pygame_gui.join()
    except Exception as e:
        print(f"An error occurred: {e}")
