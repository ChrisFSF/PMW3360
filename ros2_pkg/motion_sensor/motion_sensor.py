import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class Motion_Data:
    def __init__(self, name, data):
        self.sensor_name = name
        self.timestamp = data[0]/100.0
        self.dx = data[1]/100000.0
        self.dy = data[2]/100000.0
        self.x_dist = data[3]/100000.0
        self.y_dist = data[4]/100000.0

    def print_data(self):
        print(f"{self.sensor_name}: {self.timestamp} s: dx: {self.dx}, dy: {self.dy}, x: {self.x_dist}, y: {self.y_dist}\n")

def callback(msg):
    # print("Received array data:", msg.data)
    Sensor1 = Motion_Data("Sensor1", msg.data[0:5])
    Sensor2 = Motion_Data("Sensor2", msg.data[5:10])
    Sensor3 = Motion_Data("Sensor3", msg.data[10:])

    Sensor1.print_data()
    Sensor2.print_data()
    Sensor3.print_data()

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('esp32_motion_sensors_subscriber')

    subscriber = node.create_subscription(
        Int32MultiArray,
        'esp32_motion_sensors_publisher',
        callback,
        10  # qos depth
    )

    print('Listening for data from esp32_motion_sensors_publisher...')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_subscription(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()