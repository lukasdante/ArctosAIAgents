from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor
import torch
import json
from utils.nodes import BaseNode
from typing import List
from .joint import Joint, ParallelGripper

class Reacher(BaseNode):
    def __init__(self, joints: List[Joint]):
        super().__init__('reacher')

        try:
            # Declare parameters for the node
            self.declare_parameter('model', 'reach_best.pt', ParameterDescriptor(description='Object detection pretrained model can be .engine, .pt, or .onnx.'))

            # Load the model in evaluation mode
            model_path = f'models/{self.get_parameter("model").get_parameter_value().string_value}'
            self.model: torch.nn.Module = torch.load(model_path)
            self.model.eval()

        except Exception as e:
            self.get_logger().error(f'Failed to start Reacher node: {e}')
    
    def reach(self, state):
        
        with torch.no_grad():
            actions = self.model(state)
            
            return actions