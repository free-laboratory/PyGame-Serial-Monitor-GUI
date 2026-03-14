import numpy as np
import time

import canbus
import gvar_can
import parameter_matrices as pm

def pressure_calibration(send_child_conn, mp_actuator_status):
    try:
        # we don't know the adc to pressure mapping,
        # so we need to collect data points of (adc, pressure) pairs to do the calibration.
        # this function measure the (adc, pressure) for each actuator and save it to a file.
        
        print("Starting pressure calibration...")

        adc_vs_pressure_0_and_40psi = pm.p_list_adc_to_psi
        print(adc_vs_pressure_0_and_40psi)

        for idx, actuator_id in enumerate(gvar_can.actuator_id_list):
            print(f"Calibrating actuator {hex(actuator_id)} ({idx+1}/{len(gvar_can.actuator_id_list)})...")
            
            target_pressures = np.ones(len(gvar_can.actuator_id_list)) * 1000 # target pressure in mbar
            target_pdots = np.zeros(len(gvar_can.actuator_id_list)) # target pd
            control_bytes = np.ones(len(gvar_can.actuator_id_list)) # control byte, and we set it to 1 to enable the pressure control for all actuators
            target_ids = gvar_can.actuator_id_list

            target_pressures[idx] = 4095 # max out this one, while others are at 1000

            # send this package to the connection, which will then send the command to the can bus
            actuator_command_package = canbus.create_actuator_command_package(target_pressures, target_pdots, control_bytes, target_ids)
            send_child_conn.send(actuator_command_package)

            # wait for a few seconds to let the pressure stabilize
            # to prevent the actuator from resetting, keep sending the command package @ 10hz for 7 seconds
            for _ in range(70):
                send_child_conn.send(actuator_command_package)
                time.sleep(0.1)

            # and then read the adc value from the mp_actuator_status
            print(mp_actuator_status[actuator_id][0])
            adc_vs_pressure_0_and_40psi[idx][1] = mp_actuator_status[actuator_id][0] # the adc value is stored in the first element of the status list

            # then, set the target pressure to 0, and repeat the process to get the adc value at 0 psi
            target_pressures[idx] = 0 # set this one to 0, while others are at 1000
            actuator_command_package = canbus.create_actuator_command_package(target_pressures, target_pdots, control_bytes, target_ids)
            send_child_conn.send(actuator_command_package)


            for _ in range(70):
                send_child_conn.send(actuator_command_package)
                time.sleep(0.1)

                
            print(mp_actuator_status[actuator_id][0])
            adc_vs_pressure_0_and_40psi[idx][0] = mp_actuator_status[actuator_id][0] # the adc value is stored in the first element of the status list
            
        # after collecting the data, we can save it to a file for later use
        np.save("adc_vs_pressure_0_and_40psi.npy", adc_vs_pressure_0_and_40psi)

        print("Pressure calibration completed and saved to adc_vs_pressure_0_and_40psi.npy")
    except Exception as e:
        print(f"Error during pressure calibration: {e}")


    pass