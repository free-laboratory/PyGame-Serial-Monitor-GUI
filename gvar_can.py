gvar_manager = None

BROADCAST_CANID = 0x90

actuator_id_list = [i for i in range(0x101, 0x101 + 24)]

target_pressure = 1300
target_pdot = 0xFFFF

mp_ctrl_start_recording = 0
mp_ctrl_stop_recording = 1

mp_ctrl_start_sequence = 2

mp_ctrl_actuator_ctrl_byte = 3