"""
Actuator Benchmark Test Script

This script performs a staircase pressure test on actuators:
- Pressure sequence: 1000 -> 1500 -> 2000 -> 2500 -> 3000 -> 2500 -> 2000 -> 1500 -> 1000
- Automatically starts recording at the beginning of the test
- Automatically stops recording at the end of the test
- Can test individual actuators or all actuators simultaneously
"""

import can
import time
import datetime
import multiprocessing as mp
import gvar_can
import numpy as np
import os
from canbus import (
    write_parameters_to_actuator, 
    subroutine_CAN_handler, 
    mp_data_recoder,
    COMPORT
)


def staircase_pressure_test(send_parent_conn, mp_ctrl, actuator_ids=None, hold_duration=3.0):
    """
    Execute a staircase pressure test on specified actuators.
    
    Parameters:
    -----------
    send_parent_conn : multiprocessing.Connection
        Pipe connection to send commands to CAN handler
    mp_ctrl : multiprocessing.list
        Shared list for control flags
    actuator_ids : list of int, optional
        List of actuator IDs to test. If None, tests all actuators.
    hold_duration : float
        Duration in seconds to hold each pressure level
    """
    try:
        # Define the staircase pressure sequence
        pressure_sequence = [1200, 1600, 1900, 2400, 3200, 2800, 2400, 1900, 1300, 900]
        
        # Use all actuators if none specified
        if actuator_ids is None:
            actuator_ids = gvar_can.actuator_id_list
        
        print("=" * 80)
        print("ACTUATOR STAIRCASE PRESSURE TEST")
        print("=" * 80)
        print(f"Testing actuators: {[hex(aid) for aid in actuator_ids]}")
        print(f"Pressure sequence: {pressure_sequence}")
        print(f"Hold duration per level: {hold_duration} seconds")
        print(f"Total test duration: ~{len(pressure_sequence) * hold_duration:.1f} seconds")
        print("=" * 80)
        
        # Wait a moment before starting
        print("\nInitializing actuators to starting pressure (1000)...")
        initial_pressure = 1000
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            if aid in actuator_ids:
                # Set test actuators to initial pressure with control byte 0x01 (running)
                actuator_command_package.append([initial_pressure, 0xFFFF, 0x01, aid])
            else:
                # Keep other actuators stopped
                actuator_command_package.append([0, 0xFFFF, 0x00, aid])
        
        send_parent_conn.send(actuator_command_package)
        time.sleep(2)
        
        # Start recording
        print("\n--- STARTING RECORDING ---")
        mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 0
        time.sleep(0.5)
        
        # Execute the staircase test
        print("\n--- STARTING STAIRCASE TEST ---\n")
        for i, target_pressure in enumerate(pressure_sequence):
            print(f"Step {i+1}/{len(pressure_sequence)}: Setting pressure to {target_pressure}")
            
            # Create command package
            actuator_command_package = []
            for aid in gvar_can.actuator_id_list:
                if aid in actuator_ids:
                    # Set test actuators to target pressure
                    actuator_command_package.append([target_pressure, 0xFFFF, 0x01, aid])
                else:
                    # Keep other actuators stopped
                    actuator_command_package.append([0, 0xFFFF, 0x00, aid])
            
            send_parent_conn.send(actuator_command_package)
            
            # Hold at this pressure level
            time.sleep(hold_duration)
        
        print("\n--- STAIRCASE TEST COMPLETED ---")
        
        # Return to safe pressure
        print("\nReturning actuators to safe pressure (1000)...")
        actuator_command_package = []
        for aid in gvar_can.actuator_id_list:
            actuator_command_package.append([1000, 0xFFFF, 0x00, aid])
        send_parent_conn.send(actuator_command_package)
        time.sleep(1)
        
        # Stop recording
        print("\n--- STOPPING RECORDING ---")
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
        time.sleep(1)
        
        print("\n" + "=" * 80)
        print("TEST COMPLETED SUCCESSFULLY!")
        print("Data has been saved to the datalog folder.")
        print("=" * 80)
        
    except Exception as e:
        print(f"Error in staircase pressure test: {e}")
        # Make sure to stop recording even if there's an error
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1


def run_benchmark(actuator_numbers=None, hold_duration=3.0):
    """
    Run the complete benchmark test.
    
    Parameters:
    -----------
    actuator_numbers : list of int, optional
        List of actuator numbers (1-24) to test. If None, tests only actuator 1.
        Example: [1, 2, 3] tests actuators 0x101, 0x102, 0x103
    hold_duration : float
        Duration in seconds to hold each pressure level (default: 3.0)
    """
    # Convert actuator numbers to IDs
    if actuator_numbers is None:
        actuator_numbers = [1]  # Default to testing actuator 1
    
    actuator_ids = [0x100 + num for num in actuator_numbers]
    
    # Initialize multiprocessing manager
    gvar_can.gvar_manager = mp.Manager()
    
    # Shared queue for data logging
    mp_actuator_t_p_pdot = gvar_can.gvar_manager.Queue()
    
    # Shared control list
    mp_ctrl = gvar_can.gvar_manager.list()
    for i in range(50):
        mp_ctrl.append(0)
    
    # Shared actuator status dict
    mp_actuator_status = gvar_can.gvar_manager.dict()
    for aid in gvar_can.actuator_id_list:
        mp_actuator_status[aid] = [0, 0, 0, 0, 0, 0x00]
    
    # Create pipe for communication
    send_parent_conn, send_child_conn = mp.Pipe()
    
    # Connect to CAN bus
    print("Connecting to CAN bus...")
    bus = None
    while bus is None:
        try:
            bus = can.interface.Bus(bustype='slcan', channel=COMPORT, bitrate=1000000)
            print(f"Successfully connected to CAN bus on {COMPORT}")
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}. Retrying in 2 seconds...")
            time.sleep(2)
    
    try:
        # Write parameters to test actuators
        if actuator_ids:
            print(f"\nWriting parameters to test actuators: {[hex(aid) for aid in actuator_ids]}")
            write_parameters_to_actuator(bus, actuator_ids)
        
        # Close bus instance (will be reopened by subroutine)
        bus.shutdown()
        time.sleep(0.5)
        
        # Create processes
        print("\nStarting processes...")
        p_can_handler = mp.Process(
            target=subroutine_CAN_handler, 
            args=(send_child_conn, mp_actuator_status, mp_actuator_t_p_pdot, mp_ctrl)
        )
        
        p_data_recorder = mp.Process(
            target=mp_data_recoder, 
            args=(mp_actuator_t_p_pdot, mp_ctrl)
        )
        
        # Start processes
        p_can_handler.start()
        p_data_recorder.start()
        
        # Wait for processes to initialize
        time.sleep(5)
        
        # Run the staircase test
        staircase_pressure_test(send_parent_conn, mp_ctrl, actuator_ids, hold_duration)
        
        # Wait a moment for data to flush
        time.sleep(2)
        
        # Cleanup
        print("\nShutting down processes...")
        p_can_handler.terminate()
        p_data_recorder.terminate()
        
        p_can_handler.join(timeout=5)
        p_data_recorder.join(timeout=5)
        
        print("All processes terminated.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user!")
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
        time.sleep(1)
        
        if 'p_can_handler' in locals():
            p_can_handler.terminate()
        if 'p_data_recorder' in locals():
            p_data_recorder.terminate()
            
    except Exception as e:
        print(f"\nError during benchmark: {e}")
        
        if 'p_can_handler' in locals():
            p_can_handler.terminate()
        if 'p_data_recorder' in locals():
            p_data_recorder.terminate()


if __name__ == "__main__":
    """
    Example usage:
    
    # Test only actuator 1 (0x101)
    run_benchmark(actuator_numbers=[1], hold_duration=3.0)
    
    # Test actuators 1, 2, and 3
    run_benchmark(actuator_numbers=[1, 2, 3], hold_duration=4.0)
    
    # Test all actuators
    run_benchmark(actuator_numbers=list(range(1, 25)), hold_duration=3.0)
    """
    
    # Default: Test actuator 1 with 3 second hold at each pressure level
    run_benchmark(actuator_numbers=[8], hold_duration=3.0)
