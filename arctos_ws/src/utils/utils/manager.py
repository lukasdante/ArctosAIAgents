import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor
from interfaces.msg import NodeStatus
import time

class NodeManager(Node):
    def __init__(self):
        super().__init__('manager')

        time.sleep(2)

        # Declare parameters for the node
        self.declare_parameter('frequency', 2, ParameterDescriptor(description='Times the node status is being checked per second.'))
        self.declare_parameter('nodes', 'manager', ParameterDescriptor(description='List of all nodes that should be active and regularly checked.'))

        # Set parameters for the node
        self.check_hz = round(1 / self.get_parameter('frequency').get_parameter_value().integer_value, 2)
        self.nodes = self.get_parameter('nodes').get_parameter_value().string_value.split(',')
        self.initialized = False

        # Create publishers for the node names
        self.node_status_pub = self.create_publisher(NodeStatus, 'manager/node_status', 10)
        self.initialized_node_pub = self.create_publisher(Bool, 'manager/nodes_initialized', 10)

        # At a certain frequency, check node status
        self.create_timer(self.check_hz, self.check_node_status)

        self.get_logger().info('Manager initialized.')

    def check_node_status(self):

        active_nodes = self.get_node_names()

        # If there are inactive nodes, send a False status and a list of inactive nodes
        inactive_nodes = [node for node in self.nodes if node not in active_nodes]
        inactive_node_count = len(inactive_nodes)


        if inactive_node_count > 0:

            if self.initialized:
                # Publish node status
                self.get_logger().info("Some nodes stopped. Terminating processes.")
                msg = NodeStatus()
                msg.node_status = False
                msg.node_list = inactive_nodes
                self.node_status_pub.publish(msg)

        elif inactive_node_count == 0 and not self.initialized:
            self.get_logger().info("All nodes are active. Initializing system.")
            self.initialized = True

            msg = Bool()
            msg.data = self.initialized
            self.initialized_node_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    manager = NodeManager()

    try:
        rclpy.spin(manager)
    
    except KeyboardInterrupt:
        manager.get_logger().info('Shutting down Manager.')

    finally:
        manager.destroy_node()

if __name__ == '__main__':
    main()