from dotenv import load_dotenv
from typing import List, Iterable
import subprocess
import serial.tools.list_ports
import time
import can
import os
import torch

from .joint import Joint, ParallelGripper
from .reacher import Reacher
from utils.nodes import BaseNode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

class JointActor(BaseNode):
    def __init__(self, joint_configs: List, gripper_config: dict):
        super().__init__('actor')

        try:
            # Declare parameters
            self.declare_parameter('inference_period', 3.0, ParameterDescriptor(description='TEMP: Period for performing RL inferences.'))
            self.inference_period = self.get_parameter('inference_period').get_parameter_value().double_value

            # Initialize CAN and Arduino buses
            self.initialize_can_bus()
            # self.initialize_arduino_bus(gripper_config.get('baud_rate'))
            
            # Create the Joint nodes
            self.joints: List[Joint] = []
            for config in joint_configs:
                joint = Joint(**config, bus=self.can_bus)
                self.joints.append(joint)
            
            # Create the Gripper node
            self.gripper = ParallelGripper(**gripper_config)

            # Create the Reacher node
            # self.reacher = Reacher(joints=self.joints)

            self.timer = self.create_timer(0.1, self.spin_joints)
            self.stop_loop = False

            self.parameters_subscriber = self.create_subscription(String, 'conversation/parameters', self.preset_callback, 10)

            for joint in self.joints:
                self.get_logger().info(f'Current angle: {joint.current_angle}')

            self.get_logger().info(f'JointActor node initialized.')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize actor: {e}')

    def initialize_can_bus(self):
        # Find the CAN port
        self.can_port = self.find_port(['CAN', 'USB2CAN'])
        if self.can_port is None:
            self.get_logger().error("No CAN interface found. Please check your connection.")

        # Initialize the CAN communication
        self.change_permissions(self.can_port, "777", os.getenv('PASSWORD'))

        # Set the CAN bus
        self.can_bus = can.interface.Bus(interface='slcan', channel=self.can_port, bitrate=500000)

        self.notifier = can.Notifier(self.can_bus, [self.notifier_callback]) ## add callback

    def notifier_callback(self, msg):
        """ Publishes the notifier received messages to a topic. """
        # try:
        #     # TODO: process msg
        #     msg_pub = String()
        #     msg_pub.data = f'{msg}'
        #     self.joint_data_publisher.publish(msg_pub)
        #     self.get_logger().debug(f'Published notifier message for Joint {self.axis}.')
        # except Exception as e:
        #     self.get_logger().warn(f'Failed to publish Joint {self.axis} notifier: {e}')
        pass

    def initialize_arduino_bus(self, baud_rate):
        
        # Find the Arduino port
        self.arduino_port = self.find_port(['Arduino', 'arduino', 'ttyACM'])
        if self.arduino_port is None:
            self.get_logger().error("No Arduino serial interface found. Please check your connection.")

        # Initialize the CAN communication
        self.change_permissions(self.arduino_port, "777", os.getenv('PASSWORD'))
        
        # Set the Arduino bus
        self.arduino_bus = serial.Serial(self.arduino_port, baud_rate)

    def spin_joints(self):
        for joint in self.joints:
            rclpy.spin_once(joint, timeout_sec=0.01)

    def move_sync(self, joints: List[Joint], joint_angles, joint_velocities=[600]*6, joint_accelerations=[2]*6, type='absolute'):
        try:
            for joint, angle, velocity, acceleration in zip(joints, joint_angles, joint_velocities, joint_accelerations):
                joint.move(angle, velocity, acceleration, type=type)
        except Exception as e:
            self.get_logger().error(f'Failed to move the motors synchronously: {e}')

    def reset_position(self):
        for joint in self.joints:
            joint.home()

    def terminate_process(self):
        pass

    def preset_callback(self, msg: String):
        params = self.parse_params(msg)

        supported_predefined_actions = ['close gripper', 'open gripper', 'A', 'B', 'bow', 'turn around', 'look', 'turn left', 'turn right', 'reset position']

        if 'predefined' not in params.keys():
            return None

        action = params['predefined']

        if action not in supported_predefined_actions:
            # TODO: Callback to talker node to tell the user that the action is not currently supported.
            self.get_logger().info(f'The requested predefined action - {action} - is not currently supported.')
            return None

        self.get_logger().info(f'A predefined action - {action} - is being commenced.')

        if action == 'close gripper':
            self.gripper.move(self.gripper.close_limit)
        
        if action == 'open gripper':
            self.gripper.move(self.gripper.open_limit)

        if action == 'A':
            joint_angles = [-0x6000, -0x30000, -0x18000, 0x0, 0x3C000, 0x0]

            self.move_sync(self.joints, joint_angles)

        if action == 'B':
            joint_angles = [0x6000, -0x30000, -0x18000, 0x0, 0x3C000, 0x0]

            self.move_sync(self.joints, joint_angles)

        if action == 'reset position' or action == 'reset':
            self.reset_position()

        if action == 'B':
            joint_angles = [-0x6000, -0x30000, -0x18000, 0x0, 0x3C000, 0x0]

            self.move_sync(self.joints, joint_angles)
            pass

        if action == 'look':
            pass

        if action == 'bow':
            pass

        if action == 'turn around':
            # TODO Relative movement
            self.joints[0].move(0x1A000, 0x3000, 0x02, 'absolute')
        
        if action == 'turn left':
            # TODO Relative movement
            self.joints[0].move(0xD000, 0x3000, 0x02)
        
        if action == 'turn right':
            # TODO Relative movement
            self.joints[0].move(-0xD000, 0x3000, 0x02)

        if action == 'reach':
            state = self.collect_state()
            actions = self.reacher.reach(state)

            self.move_sync(actions)

        msg: str = msg.data
        if 'preset' in msg:
            self.get_logger().info(f'{msg.lower()}')
            if 'bow' in msg.lower():
                self.get_logger().info('IN PRESET CALLBACK: HAS PRESET: HAS BOW')
                self.move_preset([[0x0, -0x10000, -0x10000, 0x0],[0x0, 0x0, 0x0, 0x0]])
            if 'shake' in msg.lower():
                self.move_preset([[0x8000, 0x0, 0x0, 0x0],[-0x8000, 0x0, 0x0, 0x0]])

    def collect_state(self):
        """
        The state consists of:
            (6) joint positions
            (6) joint velocities
            (6) last relative joint position command
            (7) target pose
                (3) translation vector
                (4) orientation quaternion
        
        This function collects these observation terms into a single tensor
        """
        # TODO: Create a function that gets all the current joint positions

        # Gets the current joint position wrt to default
        joint_pos_rel: torch.Tensor = torch.tensor([
            self.transform_joint_position(self.joints[0].gear_ratio),
            self.transform_joint_position(),
            self.transform_joint_position(),
            self.transform_joint_position(),
            self.transform_joint_position(),
            self.transform_joint_position(),
        ])
        
        # Gets the current joint velocity wrt to default
        # We have a choice of using 0 or a constant joint velocity
        joint_vel_rel: torch.Tensor = torch.tensor()

        # Gets the last command from the memory buffer
        # TODO: Create a memory buffer that stores the last commands
        last_command: torch.Tensor = torch.tensor()

        # Gets the target pose which is a constant per reach run
        target_pose: torch.Tensor = torch.tensor()
        
        state = torch.cat((joint_pos_rel,
                          joint_vel_rel,
                          last_command,
                          target_pose),
                          dim=0).flatten()

        return state
    
    def transform_joint_position(self, joint_position, ratio):
        return joint_position * ratio

    def move_preset(self, preset_angles: List[List], delays=[5]):
        """ Moves to a preset position. """
        self.reset_position()

        while not self.stop_loop:
            for delay, preset_angle in zip(delay, preset_angles):
                self.move_sync(self.joints, preset_angle)
                # TODO: Wait until all the joints are done moving
                time.sleep(delay)


        self.reset_position()

    def change_permissions(self, file_path, permissions, password):
        """ Changes permission of the USB ports to allow automated CAN bus connection. """
        try:
            # Construct the chmod command
            command = ["sudo", "-S", "chmod", permissions, file_path]
            
            # Pass the password via stdin
            subprocess.run(
                command,
                input=f"{password}\n",
                text=True,
                capture_output=True,
                check=True
            )
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Subprocess error: {e}')

    # Locate the CAN port for USB2CAN or similar adapter
    def find_port(self, descriptors: List[str]):
        """ Finds a CAN connection from USB ports. """
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            # self.get_logger().info(f'{port.description}')
            if any([descriptor in port.description for descriptor in descriptors]):
                if 'Arduino' in descriptors:
                    self.declare_parameter('Arduino Port', port.device, ParameterDescriptor(description="Arduino port for the robot arm."))
                    self.get_logger().info(f'Found an Arduino port at {self.get_parameter("Arduino Port").get_parameter_value().string_value}')
                if 'CAN' in descriptors:
                    self.declare_parameter('CAN Port', port.device, ParameterDescriptor(description="CAN bus port for the robot arm."))
                    self.get_logger().info(f'Found a CAN port at {self.get_parameter("CAN Port").get_parameter_value().string_value}')
                return port.device
        self.get_logger().error(f'Failed to find target ports: {descriptors}.')

def main(args=None):
    load_dotenv()

    joint_configs = [
        {'axis': 'X', 'can_id': 0x01, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 2000,  'gear_ratio': 13.5,  'zero_angle': 0},
        {'axis': 'Y', 'can_id': 0x02, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 3000, 'gear_ratio': 150,   'zero_angle': 0},
        {'axis': 'Z', 'can_id': 0x03, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 3000, 'gear_ratio': 150,   'zero_angle': 0},
        {'axis': 'A', 'can_id': 0x04, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 600,  'gear_ratio': 48,    'zero_angle': 0},
        {'axis': 'B', 'can_id': 0x05, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 600,  'gear_ratio': 67.82, 'zero_angle': 0},
        {'axis': 'C', 'can_id': 0x06, 'max_ccw_rot': -90, 'max_cw_rot': 90, 'default_speed': 600,  'gear_ratio': 67.82, 'zero_angle': 0},
    ]

    gripper_config = {'axis': 'G', 'baud_rate': 9600, 'open_limit': 105, 'close_limit': 60}

    try:
        rclpy.init(args=args)

        joint_actor = JointActor(joint_configs=joint_configs, gripper_config=gripper_config)

        rclpy.spin(joint_actor)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()