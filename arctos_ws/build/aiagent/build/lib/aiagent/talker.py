from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare node parameters
        self.declare_parameter('language', 'en-US', ParameterDescriptor(description='Language of the transcription output.'))
        self.declare_parameter('gender', 'FEMALE', ParameterDescriptor(description='Gender of the text-to-speech voice agent.'))
        self.declare_parameter('accent', 'en-US-Neural2-C', ParameterDescriptor(description='Accent or voice type of the text-to-speech voice agent.'))
        self.declare_parameter('encoding_format', 'LINEAR16', ParameterDescriptor(description='Encoding format of the audio recording.'))

        # Set node parameters
        self.language = self.get_parameter('language').get_parameter_value().string_value
        self.gender = self.get_parameter('gender').get_parameter_value().string_value
        self.accent = self.get_parameter('accent').get_parameter_value().string_value
        self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
        self.token_validity = self.get_parameter('token_life').get_parameter_value().integer_value


