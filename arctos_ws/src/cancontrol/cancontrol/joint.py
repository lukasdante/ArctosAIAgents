import os
import can
import subprocess
import serial.tools.list_ports
from dotenv import load_dotenv
from collections import namedtuple
from typing import Iterable
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from utils.helpers import find_port, change_permissions

class ParallelGripper(Node):
    def __init__(self, axis, baud_rate, open_limit, close_limit, bus=None):
        super().__init__(f'{axis}')

        # Declare parameters for the node
        self.declare_parameter('axis', axis, ParameterDescriptor(description='Name of the joint.'))
        self.declare_parameter('baud_rate', baud_rate, ParameterDescriptor(description='Baud rate of the joint.'))
        self.declare_parameter('open_limit', open_limit, ParameterDescriptor(description='Max joint limit for opening the gripper.'))
        self.declare_parameter('close_limit', close_limit, ParameterDescriptor(description='Max joint limit for closing the gripper.'))

        # Set node parameters as class parameters
        self.axis = self.get_parameter('axis').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.open_limit = self.get_parameter('open_limit').get_parameter_value().integer_value
        self.close_limit = self.get_parameter('close_limit').get_parameter_value().integer_value

        try:
            self.bus = bus

            if bus is None:
                self.arduino_port = find_port(['Arduino', 'arduino', 'ttyACM'])

                if self.arduino_port is None:
                    self.get_logger().error('No Arduino serial interface found. Please check your connection.')
                change_permissions(file_path=self.arduino_port, permissions='777', password=os.getenv('PASSWORD'))

                self.bus = serial.Serial(self.arduino_port, self.baud_rate)

            self.get_logger().info(f'Joint {self.axis} initialized.')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize gripper: {e}")

    def move(self, angle):
        self.bus = serial.Serial(self.arduino_port, self.baud_rate)
        if not (self.close_limit <= angle <= self.open_limit):
            self.get_logger().warn(f'Please provide an angle between {self.close_limit} and {self.open_limit}.')
            return
        
        try:
            self.bus.write(str(angle).encode())
        except Exception as e:
            self.get_logger().error(f"Cannot move gripper: {e}")
    
    def open(self):
        self.move(self.open_limit)

    def close(self):
        self.move(self.close_limit)

class Joint(Node):
    def __init__(self, axis, can_id, max_ccw_rot, max_cw_rot, gear_ratio, zero_angle, default_speed, bus):
        super().__init__(axis)

        try:
            # Set the CAN bus
            self.bus = bus

            # If no CAN bus provided
            if bus is None:
                # Find the CAN port
                self.can_port = find_port(["CAN", "USB2CAN"])
                if self.can_port is None:
                    self.get_logger().error("No CAN interface found. Please check your connection.")
                    return
                
                # Initialize the CAN communication
                change_permissions(self.can_port, "777", os.getenv('PASSWORD'))
                self.bus = can.interface.Bus(interface='slcan', channel=self.can_port, bitrate=500000)

            # self.notifier = can.Notifier(self.bus, [self.notifier_callback]) ## add callback



            # Declare parameters for the node
            self.declare_parameter('axis', axis, ParameterDescriptor(description='Name of the joint.'))
            self.declare_parameter('can_id', can_id, ParameterDescriptor(description='Arbitration ID for CAN bus message priority.'))
            self.declare_parameter('max_ccw_rot', max_ccw_rot, ParameterDescriptor(description='Maximum clockwise joint angle.'))
            self.declare_parameter('max_cw_rot', max_cw_rot, ParameterDescriptor(description='Maximum counter-clockwise joint angle.'))
            self.declare_parameter('gear_ratio', gear_ratio, ParameterDescriptor(description='Gear ratio of the joint.'))
            self.declare_parameter('zero_angle', zero_angle, ParameterDescriptor(description='Home or zero angle of the joint.'))
            self.declare_parameter('default_speed', default_speed, ParameterDescriptor(description='Default speed (rpm) of the joint.'))
            self.declare_parameter('default_acceleration', 0x02, ParameterDescriptor(description='Default acceleration (rpm2) of the joint.'))


            # Set node parameters as class parameters
            self.axis = self.get_parameter('axis').get_parameter_value().string_value
            self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
            self.max_ccw_rot = self.get_parameter('max_ccw_rot').get_parameter_value().integer_value
            self.max_cw_rot = self.get_parameter('max_cw_rot').get_parameter_value().integer_value
            self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().integer_value
            self.zero_angle = self.get_parameter('zero_angle').get_parameter_value().integer_value
            self.default_speed = self.get_parameter('default_speed').get_parameter_value().integer_value
            self.default_acceleration = self.get_parameter('default_acceleration').get_parameter_value().integer_value

            # Create a current angle tracker
            self.current_angle = 0

            # ROS components
            self.joint_data_publisher = self.create_publisher(String, 'joint/joint_data', 10)

            # Define CAN commands 
            self.commands = {'absolute': 0xF5,
                             'relative': 0xF4,
                             'addition': 0x31,
                             'velocity': 0x32,
                            }

            self.get_logger().info(f'Joint {self.axis} initialized.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize joint: {e}')

    def crc(self, data):
        """ Calculate the CRC code for a data bytearray. """

        return sum(data + [self.can_id]) & 0xFF
    
    def send_message(self, msg):
        """ Sends a message to the CAN bus. """
        try:
            self.bus.send(msg)
            data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
            self.get_logger().debug(f"Sent message: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)")
        except Exception as e:
            self.get_logger().error(f'Failed to send message: {e}')

    # def notifier_callback(self, msg):
    #     """ Publishes the notifier received messages to a topic. """
    #     try:
    #         # TODO: process msg
    #         msg_pub = String()
    #         msg_pub.data = f'{msg}'
    #         self.joint_data_publisher.publish(msg_pub)
    #         self.get_logger().debug(f'Published notifier message for Joint {self.axis}.')
    #     except Exception as e:
    #         self.get_logger().warn(f'Failed to publish Joint {self.axis} notifier: {e}')

    def move(self, position, speed=None, acceleration=None, type='absolute'):
        """ Moves the joint to a specified position, speed, and acceleration. """
        if speed is None:
            speed = self.default_speed
        if acceleration is None:
            acceleration = self.default_acceleration

        data = [
            self.commands[type],
            (speed >> 8) & 0xFF,           # Speed high byte
            speed & 0xFF,                  # Speed low byte
            acceleration,                  # Acceleration byte
            (position >> 16) & 0xFF,       # Position high byte
            (position >> 8) & 0xFF,        # Position middle byte
            position & 0xFF                # Position low byte
        ]
        
        data.append(self.crc(data))  # Add CRC as the last byte

        # Create CAN message
        try:
            msg = can.Message(arbitration_id=self.can_id, data=data,is_extended_id=False)
            self.send_message(msg)
            
            if type == 'relative':
                self.current_angle += position
                self.get_logger().info(f'Moving to the target position {self.axis}: {self.current_angle}')
            elif type == 'absolute':
                self.current_angle = position
                self.get_logger().info(f'Moving to the target position {self.axis}: {self.current_angle}')
            
            return True
        except Exception as e:
            self.get_logger().fatal(f'Failed to move the {self.axis} joint in {type} position mode: {e}')
            return False

    def home(self):
        self.move(self.zero_angle)

    def read(self, property):
        """ Read joint position or velocity. """
        data = [
            self.commands[property],
        ]

        data.append(self.crc)

        try:
            msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
            self.send_message(msg)
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to read the joint's {type}: {e}")
            return False
        
    def read_position(self):
        self.read('addition')

    def read_velocity(self):
        self.read('velocity')



    

def main(args=None):
    load_dotenv()

    try:
        rclpy.init(args=args)

        joint = Joint()
        gripper = ParallelGripper()

        rclpy.spin(joint)
        rclpy.spin(gripper)
        
    except KeyboardInterrupt:
        joint.get_logger().info('Shutting down Joint node.')
        joint.get_logger().info('Shutting down Gripper node.')
    finally:
        joint.bus.shutdown()
        joint.destroy_node()

if __name__ == '__main__':
    main()