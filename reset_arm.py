# reset arm, set all actuator pressures to zero, and stop the pressure controller

import canbus
import gvar_can
import parameter_matrices as pm
import time
import numpy as np

if __name__ == "__main__":
    # print the file path of the adc vs pressure calibration file
    print("Loading adc vs pressure calibration data from adc_vs_pressure_0_and_40psi    .npy...")
    adc_vs_pressure_0_and_40psi = np.load("adc_vs_pressure_0_and_40psi.npy")
    print(adc_vs_pressure_0_and_40psi)