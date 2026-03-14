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
from joint_visualizer import mp_joint_visualizer

def create_canmsg_pressure_to_actuator(actuator_id, p, pdot):
    # Convert the float pressure value to 4 bytes
    pressure_bytes = map_pressure_to_4bytes(p, pdot)
    
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

def map_float_from_4bytes(byte_list):
    float_value = np.frombuffer(bytearray(byte_list), dtype=np.float32)[0]
    return float_value

def map_4bytes_from_float(float_value):
    byte_array = np.array([float_value], dtype=np.float32).tobytes()
    return list(byte_array)

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
                freq = 10 # Hz
                amplitude = 40 # pressure units
                offset = 2500 # pressure units
                duration = 3 # seconds
                send_parent_conn.send([offset, 0xFFFF])
                time.sleep(3)

                mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
                start_time = time.time()
                while time.time() - start_time < duration:
                    t = time.time() - start_time
                    pressure = int(offset + amplitude * np.sin(2 * np.pi * freq * t))
                    send_parent_conn.send([pressure, 0xFFFF])
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
                    actuator_pressures = data[1]
                    target_pressure = data[2]
                    actuator_pressure_dots = data[3]
                    target_pressure_dot = data[4]

                    date_time_vector = np.array([
                        timestamp.year,
                        timestamp.month,
                        timestamp.day,
                        timestamp.hour,
                        timestamp.minute,
                        timestamp.second,
                        timestamp.microsecond
                    ])

                    final_vector = np.concatenate((date_time_vector, np.array([actuator_pressures, target_pressure, actuator_pressure_dots, target_pressure_dot])))

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
    joint_ids    = [i for i in range(0x50, 0x100)]
    # the lisener will store the latest pressure and pressure_dot for each actuator
    actuator_pressures : Dict[int, int] = {aid: 0 for aid in actuator_ids}
    actuator_pressure_dots : Dict[int, int] = {aid: 0 for aid in actuator_ids}

    shaft_positions = np.array([0.0,0.0]) # shaft A, shaft B
    imu_ypr = np.array([0.0,0.0,0.0]) # yaw, pitch, roll

    def __init__(self, mp_joint_data):
        """Initialize listener with shared multiprocessing data"""
        super().__init__()
        self.mp_joint_data = mp_joint_data

    def on_message_received(self, msg):
        # self.current_msg = copy.deepcopy(msg)
        # self.new_msg_flag = True

        if msg.arbitration_id in self.actuator_ids:
            v1_recv, v2_recv = map_pressure_from_4bytes(msg.data)
            self.actuator_pressures[msg.arbitration_id] = v1_recv
            self.actuator_pressure_dots[msg.arbitration_id] = v2_recv


        if msg.arbitration_id in self.joint_ids:
            if msg.arbitration_id == 0x51:
                self.shaft_positions[0] = map_float_from_4bytes(msg.data[0:4])
                self.shaft_positions[1] = map_float_from_4bytes(msg.data[4:8])
            if msg.arbitration_id == 0x52:
                self.imu_ypr[0] = map_float_from_4bytes(msg.data[0:4])
                self.imu_ypr[1] = map_float_from_4bytes(msg.data[4:8])
            if msg.arbitration_id == 0x53:
                self.imu_ypr[2] = map_float_from_4bytes(msg.data[0:4])

            # Update shared multiprocessing list for visualizer
            self.mp_joint_data[0] = self.shaft_positions[0]
            self.mp_joint_data[1] = self.shaft_positions[1]
            self.mp_joint_data[2] = self.imu_ypr[0]
            self.mp_joint_data[3] = self.imu_ypr[1]
            self.mp_joint_data[4] = self.imu_ypr[2]
            
    def on_error(self, exc: Exception) -> bool:
        """Callback for error handling."""
        print(f"CAN error occurred: {exc}")
        return True # Return True to indicate the error was handled
    


def subroutine_CAN_handler(send_child_conn, mp_actuator_t_p_pdot, mp_ctrl, mp_joint_data):
    try:
        print("Starting CAN bus...")
        # try connection until success
        bus = None
        while bus is None:
            try:
                bus = can.interface.Bus(bustype='slcan', channel='COM3', bitrate=1000000)
            except Exception as e:
                print(f"Failed to connect to CAN bus: {e}. Retrying in 2 seconds...")
                time.sleep(2)
        
        
        # set up a listener, which will automatically go to the callback function once a message is received
        # inside the call back function, we will update the actuator pressure and pressure_dot values
        vema_listener = VEMA_CAN_Listener(mp_joint_data)
        notifier = can.Notifier(bus, [vema_listener])
        print("Notifier started. Waiting for CAN messages...")


        p_sent = 0
        p_dot_sent = 0
        p_value = 1300
        pdot_value = 0xFFFF

        print_time = time.time()
        log_time   = time.time()

        while True:
            # this is the joints
            for i in range(3):
                if i == 0:
                    aid = 0x50
                else:
                    aid = 0x01 # dummy id, do nothing

                canmsg = create_canmsg_pressure_to_actuator(actuator_id=aid, p=p_value, pdot=pdot_value)
                bus.send(canmsg)
                pass

            # this is the actuators
            for i in range(24):
                # time.sleep(0.01)

                if i == 0:
                    aid = 0x123
                else:
                    aid = 0x124

                if send_child_conn.poll():
                    [p_value, pdot_value] = send_child_conn.recv()
                p_value = int(p_value)
                pdot_value = int(pdot_value)

                
                canmsg = create_canmsg_pressure_to_actuator(actuator_id=aid, p=p_value, pdot=pdot_value)
                bus.send(canmsg)
                p_sent, p_dot_sent = map_pressure_from_4bytes(canmsg.data)


                if mp_ctrl[gvar_can.mp_ctrl_start_recording] == 1:
                    if time.time() - log_time > 0.0025: # 400hz
                        # put the current timestamp as datetime, actuator pressure and pressure_dot values into the shared queue
                        timestamp = datetime.datetime.now()
                        mp_actuator_t_p_pdot.put([timestamp, vema_listener.actuator_pressures.get(0x123, 0), p_value, vema_listener.actuator_pressure_dots.get(0x123, 0), pdot_value])
                        log_time = time.time()

                if time.time() - print_time > 0.1:
                    aid = 0x123
                    p_val = vema_listener.actuator_pressures.get(aid, 0)
                    pdot_val = vema_listener.actuator_pressure_dots.get(aid, 0)
                    
                    shaft_a_pos = vema_listener.shaft_positions[0]
                    shaft_b_pos = vema_listener.shaft_positions[1]
                    imu_yaw = vema_listener.imu_ypr[0]
                    imu_pitch = vema_listener.imu_ypr[1]
                    imu_roll = vema_listener.imu_ypr[2]

                    

                    print(hex(aid), p_val, p_sent, hex(pdot_val), hex(p_dot_sent), " A: ", shaft_a_pos, " B: ", shaft_b_pos,
                          " Y: ", imu_yaw, " P: ", imu_pitch, " R: ", imu_roll)

                    print_time = time.time()



    except Exception as e:
        print(f"Error in CAN handler: {e}")

    pass

def console(send_parent_conn):
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
                    elif c1 == 'b':
                        gvar_can.target_pressure = int(args[0])
                        send_parent_conn.send([gvar_can.target_pressure, gvar_can.target_pdot])
                    elif c1 == 'd':
                        pdot1 = int(args[0])
                        pdot2 = int(args[1])
                        print(pdot1, pdot2)
                        gvar_can.target_pdot = (pdot1 & 0x00FF) | ((pdot2 << 8) & 0xFF00)

                        send_parent_conn.send([gvar_can.target_pressure, gvar_can.target_pdot])

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

                    elif c1 == 'p':
                        print("here")
                        pass
                pass
            except Exception as e:
                print("invalid command, ", e)
    except Exception as e:
        print("console: ", e)



if __name__ == "__main__":
    # Initialize the multiprocessing manager and CAN bus in gvar_can
    gvar_can.gvar_manager = mp.Manager()
    # Initialize the CAN bus.
    
    # Shared list to store actuator pressures
    mp_actuator_t_p_pdot = gvar_can.gvar_manager.Queue()
    mp_ctrl = gvar_can.gvar_manager.list()

    for i in range(50):
        mp_ctrl.append(0)

    # Shared list for joint visualization data: [shaft_a, shaft_b, yaw, pitch, roll]
    mp_joint_data = gvar_can.gvar_manager.list([0.0, 0.0, 0.0, 0.0, 0.0])

    send_parent_conn, send_child_conn = mp.Pipe()

    # Receiving CAN messages
    try:
        # create the process for CAN handler
        p_can_handler = mp.Process(target=subroutine_CAN_handler, args=(send_child_conn, mp_actuator_t_p_pdot, mp_ctrl, mp_joint_data))
        # create another process for the data recoder
        p_data_recoder = mp.Process(target=mp_data_recoder, args=(mp_actuator_t_p_pdot, mp_ctrl))
        # create another process that takes in the send_parent_conn and send the target pressure and target pressure_dot to the CAN handler
        p_actuator_pressure_controller = mp.Process(target=mp_actuator_pressure_controller, args=(send_parent_conn, mp_ctrl))
        # create another process for the 3D visualizer
        p_visualizer = mp.Process(target=mp_joint_visualizer, args=(mp_joint_data,))
        
        p_data_recoder.start()
        p_can_handler.start()
        p_actuator_pressure_controller.start()
        p_visualizer.start()
        console(send_parent_conn)  # Run the main console function

        p_data_recoder.terminate()
        p_can_handler.terminate()
        p_actuator_pressure_controller.terminate()
        p_visualizer.terminate()

        p_data_recoder.join()
        p_can_handler.join()
        p_actuator_pressure_controller.join()
        p_visualizer.join()
        
    except Exception as e:
        print(f"An error occurred: {e}")
