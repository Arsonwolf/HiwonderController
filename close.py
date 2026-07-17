import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def move_servo(ser, servo_id, position, move_time_ms=500):
    packet = [
        0x55, 0x55, 
        8,          # Data length (5 + 3*1)
        0x03,       # Command (Move)
        1,          # Number of servos
        move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF,
        servo_id, position & 0xFF, (position >> 8) & 0xFF
    ]
    ser.write(bytearray(packet))

if __name__ == "__main__":
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(1) # Let the adapter wake up
            print("Moving Servo 6 to 490...")
            move_servo(ser, 6, 490, move_time_ms=500)
            time.sleep(0.6) # Wait for movement to finish
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
