import serial
import time
import sys
import tty
import termios

# --- Hardware Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def move_servos(ser, positions_dict, move_time_ms=1000):
    """Packs a dictionary of servo IDs and positions into a binary command."""
    ids = list(positions_dict.keys())
    positions = list(positions_dict.values())
    num_servos = len(ids)
    
    data_length = 5 + (3 * num_servos) 
    CMD_MULT_SERVO_MOVE = 0x03
    
    packet = [
        0x55, 0x55,
        data_length,
        CMD_MULT_SERVO_MOVE,
        num_servos,
        move_time_ms & 0xFF,
        (move_time_ms >> 8) & 0xFF
    ]
    
    for s_id, pos in zip(ids, positions):
        pos = max(0, min(1000, pos)) # Clamp bounds
        packet.append(s_id)
        packet.append(pos & 0xFF)
        packet.append((pos >> 8) & 0xFF)
        
    ser.write(bytearray(packet))

def getch():
    """Reads a single keypress from the terminal instantly without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # Set terminal to raw mode to capture individual keystrokes
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        # Always restore the terminal to normal mode
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- Main Execution ---
if __name__ == "__main__":
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(1) # Let the adapter wake up
            
            print("\n--- Gripper Keyboard Control ---")
            print("Press 'O' to open (Position 350)")
            print("Press 'L' to close (Position 470)")
            print("Press 'Q' or Ctrl+C to quit")
            print("--------------------------------\n")
            
            while True:
                # Poll for a single keystroke
                char = getch().lower()
                
                if char == 'o':
                    print("-> 'O' pressed: Moving Servo 6 to 350")
                    move_servos(ser, {6: 350}, move_time_ms=500)
                    
                elif char == 'l':
                    print("-> 'L' pressed: Moving Servo 6 to 470")
                    move_servos(ser, {6: 470}, move_time_ms=500)
                    
                elif char == 'q' or char == '\x03': # \x03 is the raw code for Ctrl+C
                    print("\nExiting...")
                    break
                    
    except serial.SerialException as e:
        print(f"\nSerial port error: {e}")