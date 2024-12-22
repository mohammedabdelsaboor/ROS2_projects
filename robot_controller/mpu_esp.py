import serial
import rclpy
from std_msgs.msg import Float32

def serial_callback(msg):
    print(f"Received data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('esp32_serial')
    
    # Create a publisher
    pub = node.create_publisher(Float32, '/acceleration_data', 10)
    
    # Open serial port (make sure it's the correct one)
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    
    while rclpy.ok():
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip().split(',')
            x = float(data[0])
            y = float(data[1])
            z = float(data[2])
            
            # Send the data to ROS 2 topic
            msg = Float32()
            msg.data = x  # You can publish x, y, or z as needed
            pub.publish(msg)

        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
