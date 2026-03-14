import canbus
import datetime
import numpy as np
import matplotlib.pyplot as plt
import os

def data_unpacker(filepath):
    # load the numpy array from the file
    data_matrix = np.load(filepath)

    timestamp = []
    actuator_pressures = []
    target_pressure = []    
    actuator_pressure_dots = []
    target_pressure_dot = []

    # unpack the data
    for lines in data_matrix:
        year = int(lines[0])
        month = int(lines[1])
        day = int(lines[2])
        hour = int(lines[3])
        minute = int(lines[4])
        second = int(lines[5])
        microsecond = int(lines[6])

        timestamp.append(datetime.datetime(year, month, day, hour, minute, second, microsecond))

        actuator_pressures.append(lines[7])
        target_pressure.append(lines[8])
        actuator_pressure_dots.append(lines[9])
        target_pressure_dot.append(lines[10])

    print("Data unpacked successfully.")
    print(f"Total records: {len(timestamp)}")
    return timestamp, actuator_pressures, target_pressure, actuator_pressure_dots, target_pressure_dot


def data_plotter(filepath):

    timestamp, actuator_pressures, target_pressure, actuator_pressure_dots, target_pressure_dot = data_unpacker(filepath)

    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(timestamp, actuator_pressures, label='Actuator Pressure')
    plt.plot(timestamp, target_pressure, label='Target Pressure', linestyle='--')
    plt.xlabel('Time')
    plt.ylabel('Pressure')
    plt.title('Actuator Pressure vs Target Pressure')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(timestamp, actuator_pressure_dots, label='Actuator Pressure Dot')
    plt.plot(timestamp, target_pressure_dot, label='Target Pressure Dot', linestyle='--')
    plt.xlabel('Time')
    plt.ylabel('Pressure Dot')
    plt.title('Actuator Pressure Dot vs Target Pressure Dot')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()

# create a data plotter that take many filepathes and plot them together
def multi_data_plotter(filepaths):
    plt.figure(figsize=(12, 6))

    for filepath in filepaths:
        timestamp, actuator_pressures, target_pressure, actuator_pressure_dots, target_pressure_dot = data_unpacker(filepath)

        # normalize the timestamp to start from zero
        # make the legend smaller

        timestamp = [(t - timestamp[0]).total_seconds() for t in timestamp]
        plt.subplot(2, 1, 1)
        plt.plot(timestamp, actuator_pressures, label=f'Actuator Pressure - {os.path.basename(filepath)[20:30]}')
        plt.plot(timestamp, target_pressure, label=f'Target Pressure - {os.path.basename(filepath)[20:30]}', linestyle='--')

        plt.subplot(2, 1, 2)
        plt.plot(timestamp, actuator_pressure_dots, label=f'Actuator Pressure Dot - {os.path.basename(filepath)[20:30]}')
        plt.plot(timestamp, target_pressure_dot, label=f'Target Pressure Dot - {os.path.basename(filepath)[20:30]}', linestyle='--')

    plt.subplot(2, 1, 1)
    plt.xlabel('Time')
    plt.ylabel('Pressure')
    plt.title('Actuator Pressure vs Target Pressure')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.xlabel('Time')
    plt.ylabel('Pressure Dot')
    plt.title('Actuator Pressure Dot vs Target Pressure Dot')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()

def actuator_pressure_plotter(filepath, actuator_number):
    """
    Plot actuator target pressure and actual pressure over time for a specific actuator.
    
    Parameters:
    -----------
    filepath : str
        Path to the .npy data file
    actuator_number : int
        Actuator number to plot (e.g., 1 for 0x101, 2 for 0x102, etc.)
        Valid range is 1-24 for actuators 0x101 to 0x118
    
    Returns:
    --------
    None (displays plot)
    """
    # Convert actuator number to actuator ID
    actuator_id = 0x100 + actuator_number
    
    # Load the numpy array from the file
    data_matrix = np.load(filepath)
    
    timestamps = []
    actual_pressures = []
    target_pressures = []
    inlet_pwms = []
    outlet_pwms = []
    
    # Parse the data
    # Data format: [year, month, day, hour, minute, second, microsecond, actuator_id, actual_pressure, inlet_pwm, outlet_pwm, target_pressure]
    for line in data_matrix:
        year = int(line[0])
        month = int(line[1])
        day = int(line[2])
        hour = int(line[3])
        minute = int(line[4])
        second = int(line[5])
        microsecond = int(line[6])
        act_id = int(line[7])
        
        # Only process data for the specified actuator
        if act_id == actuator_id:
            timestamp = datetime.datetime(year, month, day, hour, minute, second, microsecond)
            actual_pressure = line[8]
            inlet_pwm = line[9]
            outlet_pwm = line[10]
            target_pressure = line[11]
            
            timestamps.append(timestamp)
            actual_pressures.append(actual_pressure)
            inlet_pwms.append(inlet_pwm)
            outlet_pwms.append(outlet_pwm)
            target_pressures.append(target_pressure)
    
    if len(timestamps) == 0:
        print(f"No data found for actuator {hex(actuator_id)} (actuator number {actuator_number})")
        return
    
    # Normalize timestamps to start from zero (in seconds)
    time_seconds = [(t - timestamps[0]).total_seconds() for t in timestamps]
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Subplot 1: Pressure over time
    plt.subplot(2, 1, 1)
    plt.plot(time_seconds, actual_pressures, label='Actual Pressure', linewidth=2)
    plt.plot(time_seconds, target_pressures, label='Target Pressure', linestyle='--', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Pressure (ADC units)')
    plt.title(f'Actuator {hex(actuator_id)} (#{actuator_number}) - Pressure Tracking')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Subplot 2: PWM duty cycles over time
    plt.subplot(2, 1, 2)
    plt.plot(time_seconds, inlet_pwms, label='Inlet PWM', linewidth=2)
    plt.plot(time_seconds, outlet_pwms, label='Outlet PWM', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('PWM Duty Cycle')
    plt.title(f'Actuator {hex(actuator_id)} (#{actuator_number}) - PWM Duty Cycles')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print(f"\nStatistics for Actuator {hex(actuator_id)} (#{actuator_number}):")
    print(f"Total records: {len(timestamps)}")
    print(f"Duration: {time_seconds[-1]:.2f} seconds")
    print(f"Actual pressure range: {min(actual_pressures)} - {max(actual_pressures)}")
    print(f"Target pressure range: {min(target_pressures)} - {max(target_pressures)}")

if __name__ == "__main__":
    # Example usage:
    # To plot a specific actuator's data, use:
    # actuator_pressure_plotter("datalog/data_log_2025-10-06_12-45-43-921509.npy", actuator_number=1)
    
    # use the plotter to plot the newest data file in the datalog folder
    # find the newest file in the datalog folder
    datalog_folder = "datalog"
    files = os.listdir(datalog_folder)
    files = [f for f in files if f.endswith('.npy')]
    files.sort(key=lambda x: os.path.getmtime(os.path.join(datalog_folder, x)), reverse=True)
    if files:
        # Plot actuator 1 from the newest file
        newest_file = os.path.join(datalog_folder, files[0])
        print(f"Plotting data from: {newest_file}")
        actuator_pressure_plotter(newest_file, actuator_number=8)
        
        # Uncomment below to use the multi_data_plotter for comparison
        # newest_files = [os.path.join(datalog_folder, f) for f in files[:2]]
        # multi_data_plotter(newest_files)
    
