import serial
import time

# --- configuration ---
SERIAL_PORT = '/dev/ttyUSB0'  # update if your cp2102 mounted differently
BAUD_RATE = 9600

def read_servo_positions(ser, servo_ids):
    """
    requests and parses the positions of the specified servo IDs.
    """
    num_servos = len(servo_ids)
    
    # formula for length: number of parameters (N) + 2
    # params = 1 (for num_servos byte) + the number of IDs
    packet_length = num_servos + 3 
    
    CMD_MULT_SERVO_POS_READ = 0x15
    
    # build the request packet
    packet = [0x55, 0x55, packet_length, CMD_MULT_SERVO_POS_READ, num_servos] + servo_ids
    
    # flush any leftover junk in the buffer before sending
    ser.reset_input_buffer() 
    ser.write(bytearray(packet))
    
    # calculate exactly how many bytes the board will send back
    # 5 bytes for the header/config + 3 bytes (ID, Pos_Low, Pos_High) per servo
    expected_bytes = 5 + (num_servos * 3)
    
    # wait for the data to arrive (with a 0.5 sec timeout)
    start_time = time.time()
    while ser.in_waiting < expected_bytes:
        if time.time() - start_time > 0.5:
            print("timeout: didn't receive enough data back from the board.")
            return None
        time.sleep(0.01)
        
    # read the raw bytes
    data = ser.read(expected_bytes)
    
    # verify it's a valid hiwonder packet
    if data[0] == 0x55 and data[1] == 0x55 and data[3] == CMD_MULT_SERVO_POS_READ:
        reply_num_servos = data[4]
        positions = {}
        
        idx = 5 # start reading servo data at index 5
        for _ in range(reply_num_servos):
            s_id = data[idx]
            pos_l = data[idx+1]
            pos_h = data[idx+2]
            
            # bitwise OR to combine the two 8-bit pieces into one 16-bit integer
            position = pos_l | (pos_h << 8)
            positions[s_id] = position
            
            idx += 3 # jump to the next servo's data chunk
            
        return positions
    else:
        print("error: received corrupted or unrecognized packet header.")
        return None

# --- main execution ---
if __name__ == "__main__":
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(1) # let the cp2102 wake up
            
            # the IDs of the servos you want to poll
            ids_to_read = [1, 2, 3, 4, 5, 6] 
            print(f"requesting positions for servos: {ids_to_read}")
            
            pos_data = read_servo_positions(ser, ids_to_read)
            
            if pos_data:
                for s_id, pos in pos_data.items():
                    # optionally convert the raw 0-1000 value back to degrees (0-240)
                    degrees = (pos / 1000.0) * 240.0
                    print(f"servo {s_id} is at position {pos} ({degrees:.1f}°)")
            else:
                print("failed to read positions. check your rx/tx wiring.")
                
    except serial.SerialException as e:
        print(f"serial error: {e}")