import rclpy
from rclpy.node import Node

from sensor_msgs.msg import TimeReference
from std_msgs.msg import String

SYNC_TIME_REQUEST_TOPIC = "sync_time_request"
SYNC_TIME_RESPONSE_TOPIC = "sync_time_response"

class SyncRequestSubscriber(Node):

    def __init__(self):
        super().__init__('sync_time_server')
        
        self.subscription = self.create_subscription(
            String,
            SYNC_TIME_REQUEST_TOPIC,
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(TimeReference, SYNC_TIME_RESPONSE_TOPIC, 10)  #TODO QoS?
        # print ("SYNC_TIME listening on /" + SYNC_TIME_REQUEST_TOPIC + "\n")
        self.get_logger().info('I SYNC_TIME listening on /' + SYNC_TIME_REQUEST_TOPIC)

    def listener_callback(self, msg):
        t1 = self.get_clock().now()
        #self.get_logger().info('D heard!')
        #print("Heard!")
        response = TimeReference()
        response.source = msg.data
        response.time_ref = t1.to_msg()
        response.header.frame_id = "sync_time_server"
        response.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(response)


def main(args=None):
    rclpy.init(args=args)

    sync_request_subscriber = SyncRequestSubscriber()

    rclpy.spin(sync_request_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sync_request_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()