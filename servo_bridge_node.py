import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import time

class HiwonderBridgeNode(Node):
    def __init__(self):
        super().__init__('hiwonder_bridge_node')
        
        # --- Hardware Configuration ---
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 9600
        
        # 1. Map ROS joint names to Hiwonder Servo IDs
        # (Change keys to match your URDF)
        self.joint_mapping = {
            'joint_1': 1,
            'joint_2': 2,
            'joint_3': 3,
            'joint_4': 4,
            'joint_5': 5
        }
        
        # 2. Hardware Home Positions (ROS 0.0 radians = These values)
        self.home_positions = {
            1: 500,
            2: 750,
            3: 750,
            4: 270,
            5: 470
        }
        
        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Successfully opened {self.serial_port} at {self.baud_rate} baud.")
            time.sleep(1) 
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        # Subscribe to the joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Throttle control to prevent 9600 baud buffer overflow
        self.last_write_time = time.time()
        self.write_interval = 0.05 # 50ms (20Hz)

    def rad_to_servo_pos(self, radians, servo_id):
        """
        Converts ROS radians to Hiwonder integer range, centering 0.0 rad 
        on the servo's specific home position.
        """
        # Get the home position for this specific servo (default to 500 if missing)
        home = self.home_positions.get(servo_id, 500)
        
        # Convert radians to degrees
        degrees = math.degrees(radians)
        
        # HTS-35H servos have a 240 degree range mapped to 1000 units.
        # This means 1 degree = 4.166 units (1000 / 240)
        units_offset = degrees * (1000.0 / 240.0)
        
        # Apply the offset to the home position
        # NOTE: If a specific joint moves backwards compared to your RViz model, 
        # change the '+' to a '-' below for that specific servo_id.
        target_pos = int(home + units_offset)
        
        # Clamp bounds strictly between 0 and 1000 to prevent hardware damage
        target_pos = max(0, min(1000, target_pos))
        
        return target_pos

    def joint_state_callback(self, msg):
        current_time = time.time()
        
        if (current_time - self.last_write_time) < self.write_interval:
            return
            
        self.last_write_time = current_time
        
        packet_ids = []
        packet_positions = []
        
        # Parse the incoming ROS message
        for i, name in enumerate(msg.name):
            if name in self.joint_mapping:
                servo_id = self.joint_mapping[name]
                rad_pos = msg.position[i]
                
                # Convert using the new offset math
                hw_pos = self.rad_to_servo_pos(rad_pos, servo_id)
                
                packet_ids.append(servo_id)
                packet_positions.append(hw_pos)
                
        if packet_ids:
            move_time_ms = int(self.write_interval * 1000) 
            self.send_multi_servo_command(packet_ids, packet_positions, move_time_ms)

    def send_multi_servo_command(self, ids, positions, move_time_ms):
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
            packet.append(s_id)
            packet.append(pos & 0xFF)
            packet.append((pos >> 8) & 0xFF)
            
        try:
            self.ser.write(bytearray(packet))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge_node = HiwonderBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down Hiwonder Bridge.")
    finally:
        if hasattr(bridge_node, 'ser') and bridge_node.ser.is_open:
            bridge_node.ser.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()