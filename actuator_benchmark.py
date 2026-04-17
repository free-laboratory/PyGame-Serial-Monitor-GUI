"""
Actuator Benchmark Test Script

This script performs pressure benchmark tests on actuators:
- Staircase profile
- Large-jump profile
- Sine-wave profile
- Random-wave profile (slew-rate limited)
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


def _build_actuator_command_package(actuator_ids, target_pressure, run_flag=0x01):
    """Build CAN command package for all actuators using one target pressure."""
    actuator_command_package = []
    for aid in gvar_can.actuator_id_list:
        if aid in actuator_ids:
            actuator_command_package.append([int(target_pressure), 0xFFFF, run_flag, aid])
        else:
            actuator_command_package.append([0, 0xFFFF, 0x00, aid])
    return actuator_command_package


def _execute_continuous_sine_wave(
    send_parent_conn,
    actuator_ids,
    min_pressure=900,
    max_pressure=3200,
    cycles=2,
    frequency_hz=0.5,
    update_interval=0.05,
):
    """Execute a smooth sine wave by rapidly updating target pressure."""
    print("\n--- STARTING SINE WAVE TEST (CONTINUOUS) ---\n")

    center = (max_pressure + min_pressure) / 2.0
    amplitude = (max_pressure - min_pressure) / 2.0
    total_duration = cycles / frequency_hz

    start_time = time.perf_counter()
    next_report = start_time
    report_period = max(1.0, total_duration / 10.0)

    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        if elapsed >= total_duration:
            break

        angle = 2 * np.pi * frequency_hz * elapsed
        target_pressure = int(round(center + amplitude * np.sin(angle)))
        target_pressure = int(np.clip(target_pressure, min_pressure, max_pressure))

        send_parent_conn.send(_build_actuator_command_package(actuator_ids, target_pressure, run_flag=0x01))

        if now >= next_report:
            completion = min(100.0, (elapsed / total_duration) * 100.0)
            print(
                f"SINE WAVE TEST: {completion:5.1f}% "
                f"(target={target_pressure}, update={update_interval:.3f}s)"
            )
            next_report = now + report_period

        time.sleep(update_interval)

    print("\n--- SINE WAVE TEST COMPLETED ---")


def _execute_random_wave_slew_limited(
    send_parent_conn,
    actuator_ids,
    min_pressure=900,
    max_pressure=3200,
    duration=8.0,
    update_interval=0.05,
    retarget_interval=0.35,
    max_slew_rate=1200.0,
):
    """Execute random target updates while enforcing a bounded pressure slew rate."""
    print("\n--- STARTING RANDOM WAVE TEST (SLEW-LIMITED) ---\n")

    rng = np.random.default_rng()
    current_pressure = (max_pressure + min_pressure) / 2.0
    desired_pressure = current_pressure

    start_time = time.perf_counter()
    last_time = start_time
    next_retarget = start_time
    next_report = start_time
    report_period = max(1.0, duration / 10.0)

    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        if elapsed >= duration:
            break

        if now >= next_retarget:
            desired_pressure = float(rng.integers(min_pressure, max_pressure + 1))
            next_retarget = now + retarget_interval

        dt = max(now - last_time, 1e-3)
        max_delta = max_slew_rate * dt
        delta = desired_pressure - current_pressure
        if abs(delta) > max_delta:
            current_pressure += np.sign(delta) * max_delta
        else:
            current_pressure = desired_pressure

        target_pressure = int(np.clip(round(current_pressure), min_pressure, max_pressure))
        send_parent_conn.send(_build_actuator_command_package(actuator_ids, target_pressure, run_flag=0x01))

        if now >= next_report:
            completion = min(100.0, (elapsed / duration) * 100.0)
            print(
                f"RANDOM WAVE TEST: {completion:5.1f}% "
                f"(target={target_pressure}, desired={int(round(desired_pressure))}, "
                f"slew_limit={max_slew_rate:.0f}/s)"
            )
            next_report = now + report_period

        last_time = now
        time.sleep(update_interval)

    print("\n--- RANDOM WAVE TEST COMPLETED ---")


def _execute_pressure_sequence(send_parent_conn, actuator_ids, pressure_sequence, hold_duration, phase_name):
    """Execute one pressure profile phase."""
    print(f"\n--- STARTING {phase_name} ---\n")
    for i, target_pressure in enumerate(pressure_sequence):
        print(f"{phase_name} Step {i+1}/{len(pressure_sequence)}: Setting pressure to {target_pressure}")
        send_parent_conn.send(_build_actuator_command_package(actuator_ids, target_pressure, run_flag=0x01))
        time.sleep(hold_duration)
    print(f"\n--- {phase_name} COMPLETED ---")


def comprehensive_pressure_test(send_parent_conn, mp_ctrl, actuator_ids=None, hold_duration=1.0):
    """
    Execute a comprehensive pressure benchmark on specified actuators.
    
    Parameters:
    -----------
    send_parent_conn : multiprocessing.Connection
        Pipe connection to send commands to CAN handler
    mp_ctrl : multiprocessing.list
        Shared list for control flags
    actuator_ids : list of int, optional
        List of actuator IDs to test. If None, tests all actuators.
    hold_duration : float
        Base duration in seconds used to scale phase timing
    """
    try:
        staircase_sequence = [1200, 1600, 1900, 2400, 3200, 2800, 2400, 1900, 1300, 900]
        large_jump_sequence = [1000, 3200, 1000, 3200, 900, 3000, 1200, 3200, 1000]

        # Faster timing for comprehensive stress testing.
        staircase_hold = max(0.25, hold_duration * 0.40)
        jump_hold = max(0.30, hold_duration * 0.44)
        sine_update_interval = max(0.02, min(0.10, hold_duration * 0.04))
        sine_frequency_hz = 0.6 / 1.5
        sine_cycles = 3
        sine_duration = sine_cycles / sine_frequency_hz
        random_update_interval = sine_update_interval
        random_duration = max(6.0, sine_duration)
        random_retarget_interval = max(0.25, hold_duration * 0.20)
        random_max_slew_rate = 1200.0

        test_phases = [
            ("STAIRCASE TEST", staircase_sequence, staircase_hold),
            ("LARGE JUMP TEST", large_jump_sequence, jump_hold),
        ]
        total_duration = (
            sum(len(seq) * phase_hold for _, seq, phase_hold in test_phases)
            + sine_duration
            + random_duration
        )
        
        # Use all actuators if none specified
        if actuator_ids is None:
            actuator_ids = gvar_can.actuator_id_list
        
        print("=" * 80)
        print("ACTUATOR COMPREHENSIVE PRESSURE TEST")
        print("=" * 80)
        print(f"Testing actuators: {[hex(aid) for aid in actuator_ids]}")
        print("Test phases:")
        for phase_name, sequence, phase_hold in test_phases:
            print(
                f"  - {phase_name}: {len(sequence)} steps, "
                f"hold={phase_hold:.2f}s, range={min(sequence)}-{max(sequence)}"
            )
        print(
            f"  - SINE WAVE TEST: continuous, duration={sine_duration:.1f}s, "
            f"frequency={sine_frequency_hz:.2f}Hz, update={sine_update_interval:.3f}s"
        )
        print(
            f"  - RANDOM WAVE TEST: continuous, duration={random_duration:.1f}s, "
            f"retarget={random_retarget_interval:.2f}s, update={random_update_interval:.3f}s, "
            f"slew_limit={random_max_slew_rate:.0f}/s"
        )
        print(f"Estimated active test duration: ~{total_duration:.1f} seconds")
        print("=" * 80)
        
        # Wait a moment before starting
        print("\nInitializing actuators to starting pressure (1000)...")
        initial_pressure = 1000
        send_parent_conn.send(_build_actuator_command_package(actuator_ids, initial_pressure, run_flag=0x01))
        time.sleep(2)
        
        # Start recording
        print("\n--- STARTING RECORDING ---")
        mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 0
        time.sleep(0.5)
        
        # Execute all test phases in one continuous recording
        for phase_index, (phase_name, sequence, phase_hold) in enumerate(test_phases):
            _execute_pressure_sequence(send_parent_conn, actuator_ids, sequence, phase_hold, phase_name)
            print("Stabilizing at baseline pressure (1000) before next phase...")
            send_parent_conn.send(_build_actuator_command_package(actuator_ids, 1000, run_flag=0x01))
            time.sleep(0.35)

        _execute_continuous_sine_wave(
            send_parent_conn,
            actuator_ids,
            min_pressure=900,
            max_pressure=3200,
            cycles=sine_cycles,
            frequency_hz=sine_frequency_hz,
            update_interval=sine_update_interval,
        )

        print("Stabilizing at baseline pressure (1000) before random wave phase...")
        send_parent_conn.send(_build_actuator_command_package(actuator_ids, 1000, run_flag=0x01))
        time.sleep(0.35)

        _execute_random_wave_slew_limited(
            send_parent_conn,
            actuator_ids,
            min_pressure=900,
            max_pressure=3200,
            duration=random_duration,
            update_interval=random_update_interval,
            retarget_interval=random_retarget_interval,
            max_slew_rate=random_max_slew_rate,
        )
        
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
        print(f"Error in comprehensive pressure test: {e}")
        # Make sure to stop recording even if there's an error
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1


def staircase_pressure_test(send_parent_conn, mp_ctrl, actuator_ids=None, hold_duration=3.0):
    """Backward-compatible staircase-only test entry point."""
    try:
        sequence = [1200, 1600, 1900, 2400, 3200, 2800, 2400, 1900, 1300, 900]

        if actuator_ids is None:
            actuator_ids = gvar_can.actuator_id_list

        print("=" * 80)
        print("ACTUATOR STAIRCASE PRESSURE TEST")
        print("=" * 80)
        print(f"Testing actuators: {[hex(aid) for aid in actuator_ids]}")
        print(f"Pressure sequence: {sequence}")
        print(f"Hold duration per level: {hold_duration} seconds")
        print(f"Total test duration: ~{len(sequence) * hold_duration:.1f} seconds")
        print("=" * 80)

        send_parent_conn.send(_build_actuator_command_package(actuator_ids, 1000, run_flag=0x01))
        time.sleep(2)

        print("\n--- STARTING RECORDING ---")
        mp_ctrl[gvar_can.mp_ctrl_start_recording] = 1
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 0
        time.sleep(0.5)

        _execute_pressure_sequence(send_parent_conn, actuator_ids, sequence, hold_duration, "STAIRCASE TEST")

        print("\nReturning actuators to safe pressure (1000)...")
        send_parent_conn.send(_build_actuator_command_package(actuator_ids, 1000, run_flag=0x00))
        time.sleep(1)

        print("\n--- STOPPING RECORDING ---")
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1
        time.sleep(1)

        print("\n" + "=" * 80)
        print("STAIRCASE TEST COMPLETED SUCCESSFULLY!")
        print("Data has been saved to the datalog folder.")
        print("=" * 80)

    except Exception as e:
        print(f"Error in staircase pressure test: {e}")
        mp_ctrl[gvar_can.mp_ctrl_stop_recording] = 1


def run_benchmark(actuator_numbers=None, hold_duration=1.0):
    """
    Run the complete benchmark test.
    
    Parameters:
    -----------
    actuator_numbers : list of int, optional
        List of actuator numbers (1-24) to test. If None, tests only actuator 1.
        Example: [1, 2, 3] tests actuators 0x101, 0x102, 0x103
    hold_duration : float
        Base duration in seconds used to scale faster pressure timing (default: 1.0)
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
        
        # Run the comprehensive multi-profile benchmark
        comprehensive_pressure_test(send_parent_conn, mp_ctrl, actuator_ids, hold_duration)
        
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
    
    # Default: Test actuator 2 with faster comprehensive timing
    run_benchmark(actuator_numbers=[2], hold_duration=1.0)
