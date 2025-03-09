import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rclpy.parameter import Parameter
import warnings

class BaseNode(Node):
    # Suppress only the specific PyTorch warnings
    warnings.filterwarnings("ignore", category=UserWarning, module="torch.nn.modules.rnn")
    warnings.filterwarnings("ignore", category=FutureWarning, module="torch.nn.utils.weight_norm")
    warnings.simplefilter("ignore", UserWarning)

    def parse_params(self, msg: String):
        try:
            params: dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON data: {e}")
            return
        
        return params

    def change_parameter(self, msg: String):

        params = self.parse_params(msg)

        if not params:
            return
        
        self.get_logger().info(f'Parameters: {params}')

        # Supported parameters that can be changed
        supported_parameters = {
            'fps': rclpy.Parameter.Type.INTEGER,
            'gender': rclpy.Parameter.Type.STRING,
            'mode': rclpy.Parameter.Type.STRING,
            'model': rclpy.Parameter.Type.STRING,
            'threshold': rclpy.Parameter.Type.INTEGER,
            'silence_limit': rclpy.Parameter.Type.DOUBLE,
            'inference': rclpy.Parameter.Type.STRING,
        }

        status = params['change_parameter']

        self.get_logger().info(f'Status: {status}')

        # If the status of changing the parameter is False, return
        if status == 'False':
            return

        for key in params.keys():
            if key in supported_parameters.keys():
                # the parameter is the key
                parameter = key
                # the parameter data type is obtained from supported_parameters
                _type = supported_parameters[f'{key}']
                # the value is obtained from the msg
                value = params[f'{key}']

                # change parameter type if necessary
                if _type == rclpy.Parameter.Type.INTEGER:
                    value = int(value)
                if _type == rclpy.Parameter.Type.DOUBLE:
                    value = float(value)
                if _type == rclpy.Parameter.Type.STRING:
                    value = str(value)
                
                self.get_logger().info(f"Processing parameter: {key}, type: {type}, value: {value}")

        # set the parameter
        param = Parameter(parameter, _type, value)
        self.get_logger().info(f"Setting parameter: '{parameter}' to '{value}'")
        self.set_parameters([param])
        self.get_logger().info("Parameter change completed.")