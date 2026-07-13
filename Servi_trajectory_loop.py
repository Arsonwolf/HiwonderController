import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import serial
import math
import time

class HiwonderMasterBridge(Node):
    def __init__(self):
        super().__init__('hiwonder_master_bridge')
        
        # --- Hardware Configuration ---
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 9600
        
        # Map ROS joint names to Hiwonder Servo IDs
        self.joint_mapping = {
            'joint_1': 1, 'joint_2': 2, 'joint_3': 3,
            'joint_4': 4, 'joint_5': 5, 'joint_6': 6
        }
        
        # Inverse lookup (ID to Name) for publishing joint states
        self.id_to_name = {v: k for k, v in self.joint_mapping.items()}
        
        # Hardware Home Positions (ROS 0.0 radians = These values)
        self.home_positions = {
            1: 500, 2: 750, 3: 750, 
            4: 270, 5: 470, 6: 350 
        }
        
        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
            self.get_logger().info(f"Successfully opened {self.serial_port}")
            time.sleep(1) 
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        # --- Subscriptions and Publishers ---
        
        # 1. Subscribe to Trajectories (Commands from MoveIt)
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # 2. Publish Joint States (Feedback to MoveIt/RViz)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create a timer to poll the hardware at ~10Hz (0.1 seconds)
        self.poll_timer = self.create_timer(0.1, self.poll_hardware)
        
        # State management
        self.is_moving = False 

    # --- Math Helpers ---
    
    def rad_to_servo_pos(self, radians, servo_id):
        """Converts ROS radians to Hiwonder integer range (Commanding)."""
        home = self.home_positions.get(servo_id, 500)
        degrees = math.degrees(radians)
        units_offset = degrees * (1000.0 / 240.0)
        target_pos = int(home + units_offset)
        return max(0, min(1000, target_pos))

    def servo_pos_to_rad(self, current_pos, servo_id):
        """Converts Hiwonder integer range back to ROS radians (Feedback)."""
        home = self.home_positions.get(servo_id, 500)
        units_offset = current_pos - home
        degrees = units_offset / (1000.0 / 240.0)
        return math.radians(degrees)

    # --- Hardware Polling (The Publisher) ---
    
    def poll_hardware(self):
        """Requests positions from the board and publishes to /joint_states."""
        # Pause polling if we are currently flooding the serial line with trajectory commands
        if self.is_moving:
            return
            
        ids_to_poll = list(self.joint_mapping.values())
        num_servos = len(ids_to_poll)
        packet_length = num_servos + 3 
        CMD_READ = 0x15
        
        packet = [0x55, 0x55, packet_length, CMD_READ, num_servos] + ids_to_poll
        
        try:
            self.ser.reset_input_buffer()
            self.ser.write(bytearray(packet))
            
            expected_bytes = 5 + (num_servos * 3)
            start_time = time.time()
            
            # Wait briefly for response
            while self.ser.in_waiting < expected_bytes:
                if time.time() - start_time > 0.1: # 100ms timeout
                    return # Skip this publish cycle if board is busy
                time.sleep(0.005)
                
            data = self.ser.read(expected_bytes)
            
            # Parse data
            if data[0] == 0x55 and data[1] == 0x55 and data[3] == CMD_READ:
                reply_num_servos = data[4]
                
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                
                idx = 5
                for _ in range(reply_num_servos):
                    s_id = data[idx]
                    pos_l = data[idx+1]
                    pos_h = data[idx+2]
                    raw_pos = pos_l | (pos_h << 8)
                    
                    if s_id in self.id_to_name:
                        msg.name.append(self.id_to_name[s_id])
                        msg.position.append(self.servo_pos_to_rad(raw_pos, s_id))
                    
                    idx += 3
                    
                # Publish the true state to ROS
                self.state_pub.publish(msg)
                
        except Exception as e:
            pass # Ignore serial collisions during this cycle

    # --- Trajectory Execution (The Subscriber) ---
    
    def trajectory_callback(self, msg):
        """Executes a trajectory waypoint by waypoint."""
        self.is_moving = True
        prev_time_sec = 0.0
        
        for index, point in enumerate(msg.points):
            current_time_sec = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            time_diff = current_time_sec - prev_time_sec
            
            # Filter dense waypoints to prevent serial buffer overflow
            if time_diff < 0.1 and index != len(msg.points) - 1:
                continue 
                
            move_time_ms = int(time_diff * 1000)
            if move_time_ms == 0: move_time_ms = 100
                
            packet_ids, packet_positions = [], []
            
            for i, name in enumerate(msg.joint_names):
                if name in self.joint_mapping:
                    s_id = self.joint_mapping[name]
                    packet_ids.append(s_id)
                    packet_positions.append(self.rad_to_servo_pos(point.positions[i], s_id))
                    
            if packet_ids:
                self.send_multi_servo_command(packet_ids, packet_positions, move_time_ms)
                time.sleep(time_diff) # Wait for physical movement
                
            prev_time_sec = current_time_sec
            
        self.is_moving = False

    def send_multi_servo_command(self, ids, positions, move_time_ms):
        num_servos = len(ids)
        data_length = 5 + (3 * num_servos)
        packet = [0x55, 0x55, data_length, 0x03, num_servos, move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF]
        
        for s_id, pos in zip(ids, positions):
            packet.extend([s_id, pos & 0xFF, (pos >> 8) & 0xFF])
            
        try:
            self.ser.write(bytearray(packet))
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HiwonderMasterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
