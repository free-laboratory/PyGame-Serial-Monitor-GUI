import can
import time

CMD_SET_ID = 0x05
MSG_ACK    = 0xAA

COMPORT = 'COM3'  # Update this to your CAN adapter's port = 'COM31'  # Update this to your CAN adapter's port

def change_device_id(current_id, new_id):
    ctrl_id = current_id + 0x100
    stat_id = new_id + 0x300

    print(f"Connecting to Device: {hex(current_id)}")
    print(f"Requesting ID change to: {hex(new_id)}")

    try:
        # Update bustype/channel for your hardware
        bus = can.interface.Bus(bustype='slcan', channel=COMPORT, bitrate=1000000)
    except Exception as e:
        print(f"[ERROR] Could not open CAN bus: {e}")
        return

    try:
        # Pack the new ID into bytes 1 and 2
        high_byte = (new_id >> 8) & 0xFF
        low_byte  = new_id & 0xFF
        
        msg = can.Message(
            arbitration_id=ctrl_id, 
            data=[CMD_SET_ID, high_byte, low_byte], 
            is_extended_id=False
        )
        
        print("Sending command...")
        bus.send(msg)

        # Wait for the ACK
        end_time = time.time() + 2.0
        success = False
        while time.time() < end_time:
            ack_msg = bus.recv(0.1)
            if ack_msg and ack_msg.arbitration_id == stat_id:
                if ack_msg.data[0] == MSG_ACK:
                    success = True
                    break

        if success:
            print("✅ SUCCESS: Device acknowledged the new ID.")
            print("The device is now rebooting. It will come back online as", hex(new_id))
        else:
            print("❌ ERROR: No ACK received. Did the device get the message?")

    finally:
        bus.shutdown()

if __name__ == "__main__":
    # Example: Change a factory-fresh board (0x102) to the Left Arm Actuator (0x103)
    CURRENT_FACTORY_ID = 0x101
    NEW_TARGET_ID      = 0x102
    
    change_device_id(CURRENT_FACTORY_ID, NEW_TARGET_ID)