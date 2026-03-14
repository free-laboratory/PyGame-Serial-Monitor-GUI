import numpy as np

p101_adc_to_psi = [714, 2865]
p102_adc_to_psi = [686, 2755]
p103_adc_to_psi = [718, 2915]
p104_adc_to_psi = [708, 2835]
p105_adc_to_psi = [708, 2860]
p106_adc_to_psi = [700, 2840]
p107_adc_to_psi = [692, 2785]
p108_adc_to_psi = [696, 2785]

p109_adc_to_psi = [725, 2956]
p10A_adc_to_psi = [726, 2980]
p10B_adc_to_psi = [740, 2991]
p10C_adc_to_psi = [663, 2655]
p10D_adc_to_psi = [734, 3005]
p10E_adc_to_psi = [711, 2920]
p10F_adc_to_psi = [731, 2975]
p110_adc_to_psi = [731, 2984]

p111_adc_to_psi = [715, 2928]
p112_adc_to_psi = [703, 2870]
p113_adc_to_psi = [713, 2913]
p114_adc_to_psi = [714, 2898]
p115_adc_to_psi = [725, 2958]
p116_adc_to_psi = [717, 2926]
p117_adc_to_psi = [711, 2890]
p118_adc_to_psi = [740, 3010]

p_list_adc_to_psi = {
    0x101 : p101_adc_to_psi,
    0x102 : p102_adc_to_psi,
    0x103 : p103_adc_to_psi,
    0x104 : p104_adc_to_psi,
    0x105 : p105_adc_to_psi,
    0x106 : p106_adc_to_psi,
    0x107 : p107_adc_to_psi,
    0x108 : p108_adc_to_psi,
	0x109 : p109_adc_to_psi,
	0x10A : p10A_adc_to_psi,
	0x10B : p10B_adc_to_psi,
	0x10C : p10C_adc_to_psi,
	0x10D : p10D_adc_to_psi,
	0x10E : p10E_adc_to_psi,
	0x10F : p10F_adc_to_psi,
	0x110 : p110_adc_to_psi,
	0x111 : p111_adc_to_psi,
	0x112 : p112_adc_to_psi,
	0x113 : p113_adc_to_psi,
	0x114 : p114_adc_to_psi,
	0x115 : p115_adc_to_psi,
	0x116 : p116_adc_to_psi,
	0x117 : p117_adc_to_psi,
	0x118 : p118_adc_to_psi,
}

def adc_to_psi(actuator_id, adc_value):
	if actuator_id not in p_list_adc_to_psi:
		print(f"Actuator ID {hex(actuator_id)} not found in calibration data.")
	else:
		adc_0, adc_40 = p_list_adc_to_psi[actuator_id]

		# linear interpolation
		psi_value = (adc_value - adc_0) / (adc_40 - adc_0) * 40
		return psi_value


p101_inlet = np.array([
	[510, 490, 470],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p101_outlet = np.array([
	[510, 490, 480],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p102_inlet = np.array([
	[510, 490, 470],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p102_outlet = np.array([
	[510, 490, 480],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p103_inlet = np.array([
	[510, 490, 470],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p103_outlet = np.array([
	[525, 500, 490],  # parameter index when receiving from host: 9, 10, 11
	[525, 500, 490],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p104_inlet = np.array([
	[490, 480, 470],  # parameter index when receiving from host: 9, 10, 11
	[490, 480, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p104_outlet = np.array([
	[500, 490, 480],  # parameter index when receiving from host: 9, 10, 11
	[500, 490, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p105_inlet = np.array([
	[510, 490, 470],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p105_outlet = np.array([
	[510, 490, 480],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p106_inlet = np.array([
	[450, 435, 420],  # parameter index when receiving from host: 9, 10, 11
	[450, 435, 420],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p106_outlet = np.array([
	[490, 485, 480],  # parameter index when receiving from host: 9, 10, 11
	[490, 485, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p107_inlet = np.array([
	[470, 450, 420],  # parameter index when receiving from host: 9, 10, 11
	[470, 450, 420],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p107_outlet = np.array([
	[510, 490, 480],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 480],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p108_inlet = np.array([
	[510, 490, 470],  # parameter index when receiving from host: 9, 10, 11
	[510, 490, 470],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

p108_outlet = np.array([
	[550, 520, 500],  # parameter index when receiving from host: 9, 10, 11
	[550, 520, 500],  # parameter index when receiving from host: 12, 13, 14
	[1023, 1023, 1023]  # parameter index when receiving from host: 15, 16, 17
], dtype=np.uint16)

# create a dictionary, key is 0x101 ~ 0x108, value is a tuple (inlet_matrix, outlet_matrix)
param_dict = {
    0x101: (p101_inlet, p101_outlet),
    0x102: (p102_inlet, p102_outlet),
    0x103: (p103_inlet, p103_outlet),
    0x104: (p104_inlet, p104_outlet),
    0x105: (p105_inlet, p105_outlet),
    0x106: (p106_inlet, p106_outlet),
    0x107: (p107_inlet, p107_outlet),
    0x108: (p108_inlet, p108_outlet)
}