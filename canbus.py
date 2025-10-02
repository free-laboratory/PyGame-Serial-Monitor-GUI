import can
import time
import multiprocessing as mp
import gvar_can
import re
import numpy as np
import copy
from typing import Dict

def create_canmsg_pressure_to_actuator(actuator_id, p, pdot):
    # Convert the float pressure value to 4 bytes
    pressure_bytes = map_pressure_to_4bytes(p, pdot)
    
    # Create a CAN message with the actuator ID and pressure bytes
    return create_can_message(actuator_id, pressure_bytes)
    
def map_pressure_to_4bytes(p, pdot):
    b1 = p & 0x000000FF
    b2 = (p & 0x0000FF00) >> 8
    b3 = (pdot & 0x00FF0000) >> 16
    b4 = (pdot & 0xFF000000) >> 24
    return [b1, b2, b3, b4]

def map_pressure_from_4bytes(byte_list):
    v1_uint16_t = (byte_list[0] << 0) | (byte_list[1] << 8)
    v2_uint16_t = (byte_list[2] << 0) | (byte_list[3] << 8)
    return v1_uint16_t, v2_uint16_t

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



def subroutine_send_can_messages(send_parent_conns):
    try:
        time.sleep(2)
        p_dot_val = 0xFFFFFFFF
        counter_value = 0x0000
        while True:
            counter_value += 1
            if counter_value > 0xFFFF:
                counter_value = 0x0000

            p_dot_val = counter_value << 16


            canmsg = create_canmsg_pressure_to_actuator(actuator_id=0x123, p=200, pdot=p_dot_val)
            send_parent_conns.send(canmsg)
            time.sleep(0.001)
    except Exception as e:
        print(f"Error in sending CAN messages: {e}")


# Define a custom listener class
class VEMA_CAN_Listener(can.Listener):
    # current_msg = None
    # new_msg_flag = False
    
    # currently, there will be 24 actuators, where the id range from 0x101 to 0x124
    actuator_ids = [i for i in range(0x101, 0x125)]
    # the lisener will store the latest pressure and pressure_dot for each actuator
    actuator_pressures : Dict[int, int] = {aid: 0 for aid in actuator_ids}
    actuator_pressure_dots : Dict[int, int] = {aid: 0 for aid in actuator_ids}

    def on_message_received(self, msg):
        # self.current_msg = copy.deepcopy(msg)
        # self.new_msg_flag = True

        if msg.arbitration_id in self.actuator_ids:
            v1_recv, v2_recv = map_pressure_from_4bytes(msg.data)
            self.actuator_pressures[msg.arbitration_id] = v1_recv
            self.actuator_pressure_dots[msg.arbitration_id] = v2_recv


    def on_error(self, exc: Exception) -> bool:
        """Callback for error handling."""
        print(f"CAN error occurred: {exc}")
        return True # Return True to indicate the error was handled
    


def subroutine_CAN_handler(send_child_conn):
    try:
        print("Starting CAN bus...")
        bus = can.interface.Bus(bustype='slcan', channel='COM9', bitrate=1000000)
        # set up a listener, which will automatically go to the callback function once a message is received
        # inside the call back function, we will update the actuator pressure and pressure_dot values
        vema_listener = VEMA_CAN_Listener()
        notifier = can.Notifier(bus, [vema_listener])
        print("Notifier started. Waiting for CAN messages...")


        p_sent = 0
        p_dot_sent = 0

        p_dot_sent_prev = 0

        print_time = time.time()

        p_value = 200
        pdot_value = 0xFFFFFFFF
        counter_value = 0x0000

        while True:
            counter_value += 1
            if counter_value > 0xFFFF:
                counter_value = 0x0000

            pdot_value = counter_value << 16

            for i in range(24):
                if i == 0:
                    aid = 0x123
                else:
                    aid = 0x124

                canmsg = create_canmsg_pressure_to_actuator(actuator_id=aid, p=p_value, pdot=pdot_value)
                bus.send(canmsg)
                p_sent, p_dot_sent = map_pressure_from_4bytes(canmsg.data)


                if time.time() - print_time > 1.0:
                    aid = 0x123
                    p_val = vema_listener.actuator_pressures.get(aid, 0)
                    pdot_val = vema_listener.actuator_pressure_dots.get(aid, 0)
                    
                    print(hex(aid), p_val, p_sent, hex(pdot_val), hex(p_dot_sent), p_dot_sent - p_dot_sent_prev)
                    p_dot_sent_prev = p_dot_sent

                    print_time = time.time()



    except Exception as e:
        print(f"Error in CAN handler: {e}")

    pass

def console():
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
    mp_actuator_pressure = gvar_can.gvar_manager.list()
    mp_actuator_pressure.append(0.0)  # Initialize with a default pressure value


    send_parent_conn, send_child_conn = mp.Pipe()

    # Receiving CAN messages
    try:
            
        p_can_handler = mp.Process(target=subroutine_CAN_handler, args=(send_child_conn,))
        p_actuator_pressure_updater = mp.Process(target=subroutine_send_can_messages, args=(send_parent_conn,))

        p_can_handler.start()
        p_actuator_pressure_updater.start()

        console()  # Run the main console function

        p_can_handler.terminate()
        p_actuator_pressure_updater.terminate()

        p_can_handler.join()
        p_actuator_pressure_updater.join()  

    except Exception as e:
        print(f"An error occurred: {e}")
