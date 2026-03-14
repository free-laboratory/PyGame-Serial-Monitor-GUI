import can
import time
import os
import math

# --- OTA PROTOCOL DEFINITIONS ---
CMD_START = 0x01
CMD_END   = 0x02
MSG_ACK   = 0xAA
MSG_NACK  = 0xFF

FRAMES_PER_BLOCK = 128
DATA_PER_FRAME   = 7

COMPORT = 'COM31'  # Update this to your CAN adapter's port

def wait_for_ack(bus, stat_id, timeout=2.0):
    """Listens on the CAN bus for an ACK or NACK from the ESP32."""
    end_time = time.time() + timeout
    while time.time() < end_time:
        msg = bus.recv(0.1) # Non-blocking read with short timeout
        if msg and msg.arbitration_id == stat_id:
            if msg.data[0] == MSG_ACK:
                return True, 0
            elif msg.data[0] == MSG_NACK:
                req_seq = msg.data[1] if msg.dlc > 1 else 0
                return False, req_seq
    return False, -1 # Timeout


def perform_can_ota_on_bus(bus, device_can_id, bin_filepath, log_fn=print):
    """Runs OTA on an already-open CAN bus. Returns True on success."""
    ctrl_id = device_can_id + 0x100
    data_id = device_can_id + 0x200
    stat_id = device_can_id + 0x300

    if not os.path.exists(bin_filepath):
        log_fn(f"[ERROR] Firmware file not found: {bin_filepath}")
        return False

    file_size = os.path.getsize(bin_filepath)
    log_fn(f"--- Starting CAN OTA for Device ID: {hex(device_can_id)} ---")
    log_fn(f"Using CTRL: {hex(ctrl_id)}, DATA: {hex(data_id)}, STAT: {hex(stat_id)}")
    log_fn(f"Firmware Size: {file_size} bytes")

    # 1. Send START command
    log_fn("Sending START command...")
    bus.send(can.Message(arbitration_id=ctrl_id, data=[CMD_START], is_extended_id=False))

    success, _ = wait_for_ack(bus, stat_id, timeout=3.0)
    if not success:
        log_fn("[ERROR] No ACK received for START command. Is the device connected?")
        return False

    log_fn("Device acknowledged START. Halting hardware and preparing flash...")
    time.sleep(0.5)

    # 2. Read firmware and chunk it
    with open(bin_filepath, 'rb') as f:
        firmware_data = f.read()

    total_blocks = math.ceil(file_size / (FRAMES_PER_BLOCK * DATA_PER_FRAME))

    # 3. Send data blocks
    for block_idx in range(total_blocks):
        start_byte = block_idx * (FRAMES_PER_BLOCK * DATA_PER_FRAME)
        end_byte = start_byte + (FRAMES_PER_BLOCK * DATA_PER_FRAME)
        block_data = firmware_data[start_byte:end_byte]

        total_frames = math.ceil(len(block_data) / DATA_PER_FRAME)
        log_fn(f"Sending Block {block_idx + 1}/{total_blocks} ({total_frames} frames)...")

        retry_count = 0
        while retry_count < 3:
            for seq in range(total_frames):
                frame_start = seq * DATA_PER_FRAME
                frame_data = block_data[frame_start : frame_start + DATA_PER_FRAME]

                payload = [seq] + list(frame_data)
                msg = can.Message(arbitration_id=data_id, data=payload, is_extended_id=False)
                bus.send(msg)
                time.sleep(0.0005)

            if total_frames < FRAMES_PER_BLOCK:
                log_fn("Partial block sent")
                break

            success, req_seq = wait_for_ack(bus, stat_id, timeout=2.0)
            if success:
                log_fn("Block ACK received")
                break

            retry_count += 1
            log_fn(f"Block NACK/timeout. Retrying ({retry_count}/3), requested seq={req_seq}")

        if retry_count >= 3:
            log_fn("[ERROR] Failed to send block after 3 retries. Aborting OTA.")
            return False

    # 4. Send END command
    log_fn("All data sent. Sending END command...")
    bus.send(can.Message(arbitration_id=ctrl_id, data=[CMD_END], is_extended_id=False))

    success, _ = wait_for_ack(bus, stat_id, timeout=4.0)
    if not success:
        log_fn("[ERROR] OTA failed during final verification.")
        return False

    log_fn("========================================")
    log_fn("OTA UPDATE SUCCESSFUL")
    log_fn("The ESP32 is now rebooting.")
    log_fn("========================================")
    return True


def perform_can_ota(device_can_id, bin_filepath):
    """Standalone OTA wrapper that opens/closes the CAN bus."""
    try:
        bus = can.interface.Bus(bustype='slcan', channel=COMPORT, bitrate=1000000)
    except Exception as e:
        print(f"[ERROR] Could not open CAN bus: {e}")
        return False

    try:
        return perform_can_ota_on_bus(bus, device_can_id, bin_filepath, log_fn=print)
    finally:
        bus.shutdown()

if __name__ == "__main__":
    # Point this to your compiled firmware path
    FIRMWARE_BIN = "Valve_not_embedded_XL.bin"
    
    # Enter the DEVICE_CAN_ID exactly as it is in your C code
    TARGET_DEVICE = 0x102 
    
    perform_can_ota(TARGET_DEVICE, FIRMWARE_BIN)