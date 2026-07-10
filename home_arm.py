import serial
import time

# --- Hardware Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# Your specific home positions (ID: Position)
HOME_POSITIONS = {
    1: 500,
    2: 750,
    3: 750,
    4: 270,
    5: 470
}

def move_to_home(ser, positions_dict, move_time_ms=2000):
    """
    Takes a dictionary of servo IDs and positions, packs them into a single 
    Hiwonder multi-move command, and sends them.
    """
    # Extract IDs and target positions from the dictionary
    ids = list(positions_dict.keys())
    positions = list(positions_dict.values())
    
    num_servos = len(ids)
    
    # Calculate packet length: 5 base parameter bytes + (3 bytes per servo)
    data_length = 5 + (3 * num_servos) 
    CMD_MULT_SERVO_MOVE = 0x03
    
    # Build the packet header and configuration
    packet = [
        0x55, 0x55,                  # Header
        data_length,                 # Length of data payload
        CMD_MULT_SERVO_MOVE,         # Command Code
        num_servos,                  # Number of servos to move
        move_time_ms & 0xFF,         # Move Time (Lower 8 bits)
        (move_time_ms >> 8) & 0xFF   # Move Time (Higher 8 bits)
    ]
    
    # Append the specific ID and Position bytes for every servo
    for s_id, pos in zip(ids, positions):
        # Clamp position just to be safe
        pos = max(0, min(1000, pos))
        
        packet.append(s_id)
        packet.append(pos & 0xFF)         # Position (Lower 8 bits)
        packet.append((pos >> 8) & 0xFF)  # Position (Higher 8 bits)
        
    byte_packet = bytearray(packet)
    
    print(f"Homing {num_servos} servos over {move_time_ms}ms...")
    
    # Print the raw hex string for debugging
    hex_str = ' '.join([f"0x{b:02X}" for b in byte_packet])
    print(f"Packet: {hex_str}")
    
    # Fire the command
    ser.write(byte_packet)

# --- Main Execution ---
if __name__ == "__main__":
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(1) # Allow CP2102 adapter to wake up
            
            print("Initiating Safe Homing Sequence...")
            
            # Send the command using your mapped dictionary
            move_to_home(ser, HOME_POSITIONS, move_time_ms=2000)
            
            # Wait for the physical movement to finish (2 seconds + a small buffer)
            time.sleep(2.5) 
            
            print("Arm is now at Home Position.")
            
    except serial.SerialException as e:
        print(f"Serial port error: {e}")